/*
 * 1-Wire bus master (bit-banged on one GPIO per bus).
 *
 * Origin: esp-open-rtos extras/onewire (MIT, (c) 2014 zeroday nodemcu.com), itself based on the
 * Arduino OneWire library (Jim Studt, Paul Stoffregen and many contributors). Search algorithm and
 * CRC8 table: Dallas Semiconductor / Maxim (portions (C) 2000 Dallas Semiconductor Corporation,
 * see LICENSE). Updated for ESP-IDF 4.x / 5.x and ESP8266_RTOS_SDK by Aram Vartanyan, 2026.
 *
 * Timing follows the DS18B20 datasheet (AC characteristics) with the standard values of
 * Maxim application note 126:
 *   reset:    low 480 us, release, sample presence at 70 us, then 410 us more (tRSTH >= 480 us)
 *   write 1:  low 6 us, high 64 us (tLOW1 1..15 us, tSLOT 60..120 us)
 *   write 0:  low 60 us, high 10 us (tLOW0 60..120 us, tREC >= 1 us)
 *   read:     low 3 us, release, sample 10 us later (< 15 us, tRDV), high 55 us
 * Interrupts are disabled only inside one time slot (<= 70 us), never for a whole byte.
 */

#include <string.h>
#include "onewire.h"

/* ------------------------------------------------------------- platform */

#ifdef ESP_PLATFORM
/* ESP-IDF (all ESP32 chips) and ESP8266_RTOS_SDK: both define ESP_PLATFORM */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#if CONFIG_IDF_TARGET_ESP8266
#include "rom/ets_sys.h"
#define OW_DELAY_US(us)     ets_delay_us(us)
#define OW_ENTER_CRITICAL() taskENTER_CRITICAL()
#define OW_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#define OW_MODE_OD          GPIO_MODE_OUTPUT_OD     /* the ESP8266 reads an output pin as well */
#define OW_MODE_PP          GPIO_MODE_OUTPUT
#else
#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
#include "esp_rom_sys.h"
#define OW_DELAY_US(us)     esp_rom_delay_us(us)
#else
#include "esp32/rom/ets_sys.h"
#define OW_DELAY_US(us)     ets_delay_us(us)
#endif
static portMUX_TYPE s_ow_mux = portMUX_INITIALIZER_UNLOCKED;
#define OW_ENTER_CRITICAL() portENTER_CRITICAL(&s_ow_mux)
#define OW_EXIT_CRITICAL()  portEXIT_CRITICAL(&s_ow_mux)
/* ESP32 chips read a pin only with its input enabled */
#define OW_MODE_OD          GPIO_MODE_INPUT_OUTPUT_OD
#define OW_MODE_PP          GPIO_MODE_INPUT_OUTPUT
#endif

#define OW_READ(pin)        gpio_get_level((gpio_num_t)(pin))
#define OW_WRITE(pin, v)    gpio_set_level((gpio_num_t)(pin), (v))

#ifndef CONFIG_ONEWIRE_INTERNAL_PULLUP
#define CONFIG_ONEWIRE_INTERNAL_PULLUP 0
#endif

/* Configured once per pin (gpio_config() logs every call on ESP-IDF); afterwards only the
 * direction changes between open drain (bus) and push-pull (strong pull-up). */
static uint64_t s_configured;

static void ow_pin_od(int pin)
{
    if (!(s_configured & (1ULL << pin))) {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << pin,
            .mode = OW_MODE_OD,
            .pull_up_en = CONFIG_ONEWIRE_INTERNAL_PULLUP ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        OW_WRITE(pin, 1);
        gpio_config(&io);
        s_configured |= 1ULL << pin;
    } else {
        gpio_set_direction((gpio_num_t)pin, OW_MODE_OD);
    }
    OW_WRITE(pin, 1);
}

static void ow_pin_pp(int pin)
{
    OW_WRITE(pin, 1);
    gpio_set_direction((gpio_num_t)pin, OW_MODE_PP);
}

#else
/* esp-open-rtos */
#include "FreeRTOS.h"
#include "task.h"
#include "esp/gpio.h"
#include "espressif/esp_misc.h"
#define OW_DELAY_US(us)     sdk_os_delay_us(us)
#define OW_ENTER_CRITICAL() taskENTER_CRITICAL()
#define OW_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#define OW_READ(pin)        gpio_read(pin)
#define OW_WRITE(pin, v)    gpio_write((pin), (v))

static void ow_pin_od(int pin)
{
    gpio_enable(pin, GPIO_OUT_OPEN_DRAIN);
    gpio_write(pin, 1);
}

static void ow_pin_pp(int pin)
{
    gpio_write(pin, 1);
    gpio_enable(pin, GPIO_OUTPUT);
}
#endif

/* ----------------------------------------------------------------- timing */

#define ONEWIRE_SELECT_ROM 0x55
#define ONEWIRE_SKIP_ROM   0xcc
#define ONEWIRE_SEARCH     0xf0

#define T_RESET_LOW     480     /* tRSTL >= 480 us */
#define T_PRESENCE      70      /* sample the presence pulse (tPDHIGH 15..60 + tPDLOW 60..240) */
#define T_RESET_REST    410     /* rest of tRSTH >= 480 us */
#define T_W1_LOW        6       /* tLOW1 1..15 us: short, so a weak pull-up still has time */
#define T_W1_HIGH       64
#define T_W0_LOW        60      /* tLOW0 60..120 us */
#define T_W0_HIGH       10
#define T_R_LOW         3       /* > 1 us, as short as possible (datasheet fig. 18) */
#define T_R_SAMPLE      10      /* sample before 15 us from the falling edge (tRDV) */
#define T_R_REST        55

// Waits up to `max_wait` microseconds for the specified pin to go high.
// Returns true if successful, false if the bus never comes high (likely
// shorted).
static inline bool _onewire_wait_for_bus(int pin, int max_wait)
{
    bool state;
    for (int i = 0; i < ((max_wait + 4) / 5); i++) {
        if (OW_READ(pin)) {
            break;
        }
        OW_DELAY_US(5);
    }
    state = OW_READ(pin);
    // Wait an extra 1us to make sure the devices have an adequate recovery
    // time (tREC) before we drive things low again.
    OW_DELAY_US(1);
    return state;
}

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return false;
//
// Returns true if a device asserted a presence pulse, false otherwise.
//
bool onewire_reset(int pin)
{
    bool r;

    ow_pin_od(pin);
    // wait until the wire is high... just in case
    if (!_onewire_wait_for_bus(pin, 250)) {
        return false;
    }

    OW_WRITE(pin, 0);
    OW_DELAY_US(T_RESET_LOW);   // longer is harmless (no parasite power at > 960 us here)

    OW_ENTER_CRITICAL();
    OW_WRITE(pin, 1);           // allow it to float
    OW_DELAY_US(T_PRESENCE);
    r = !OW_READ(pin);
    OW_EXIT_CRITICAL();

    // complete tRSTH: the devices end their presence pulse within 300 us
    OW_DELAY_US(T_RESET_REST);
    if (!_onewire_wait_for_bus(pin, 10)) {
        return false;
    }
    return r;
}

static bool _onewire_write_bit(int pin, bool v)
{
    if (!_onewire_wait_for_bus(pin, 10)) {
        return false;
    }
    OW_ENTER_CRITICAL();
    OW_WRITE(pin, 0);           // drive output low
    if (v) {
        OW_DELAY_US(T_W1_LOW);
        OW_WRITE(pin, 1);       // allow output high
        OW_EXIT_CRITICAL();
        OW_DELAY_US(T_W1_HIGH);
    } else {
        OW_DELAY_US(T_W0_LOW);
        OW_WRITE(pin, 1);       // allow output high
        OW_EXIT_CRITICAL();
        OW_DELAY_US(T_W0_HIGH);
    }
    return true;
}

static int _onewire_read_bit(int pin)
{
    int r;

    if (!_onewire_wait_for_bus(pin, 10)) {
        return -1;
    }
    OW_ENTER_CRITICAL();
    OW_WRITE(pin, 0);
    OW_DELAY_US(T_R_LOW);
    OW_WRITE(pin, 1);           // let pin float, pull up will raise
    OW_DELAY_US(T_R_SAMPLE);
    r = OW_READ(pin);           // must sample within 15us of start
    OW_EXIT_CRITICAL();
    OW_DELAY_US(T_R_REST);

    return r;
}

int onewire_read_bit(int pin)
{
    return _onewire_read_bit(pin);
}

// Write a byte. The writing code uses open-drain mode and expects the pullup
// resistor to pull the line high when not driven low.  If you need strong
// power after the write (e.g. DS18B20 in parasite power mode) then call
// onewire_write_power() instead.
//
bool onewire_write(int pin, uint8_t v)
{
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1) {
        if (!_onewire_write_bit(pin, (bitMask & v))) {
            return false;
        }
    }
    return true;
}

bool onewire_write_power(int pin, uint8_t v)
{
    for (uint8_t bitMask = 0x01; bitMask != 0x80; bitMask <<= 1) {
        if (!_onewire_write_bit(pin, (bitMask & v))) {
            return false;
        }
    }
    // last bit and strong pull-up in one critical section: the bus is driven high right at the
    // end of the slot (datasheet: within 10 us after the command)
    if (!_onewire_wait_for_bus(pin, 10)) {
        return false;
    }
    bool last = v & 0x80;
    OW_ENTER_CRITICAL();
    OW_WRITE(pin, 0);
    OW_DELAY_US(last ? T_W1_LOW : T_W0_LOW);
    OW_WRITE(pin, 1);
    ow_pin_pp(pin);
    OW_EXIT_CRITICAL();
    return true;
}

bool onewire_write_bytes(int pin, const uint8_t *buf, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (!onewire_write(pin, buf[i])) {
            return false;
        }
    }
    return true;
}

// Read a byte
//
int onewire_read(int pin)
{
    int r = 0;

    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1) {
        int bit = _onewire_read_bit(pin);
        if (bit < 0) {
            return -1;
        } else if (bit) {
            r |= bitMask;
        }
    }
    return r;
}

bool onewire_read_bytes(int pin, uint8_t *buf, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        int b = onewire_read(pin);
        if (b < 0) {
            return false;
        }
        buf[i] = b;
    }
    return true;
}

bool onewire_select(int pin, onewire_addr_t addr)
{
    if (!onewire_write(pin, ONEWIRE_SELECT_ROM)) {
        return false;
    }
    for (uint8_t i = 0; i < 8; i++) {
        if (!onewire_write(pin, addr & 0xff)) {
            return false;
        }
        addr >>= 8;
    }
    return true;
}

bool onewire_skip_rom(int pin)
{
    return onewire_write(pin, ONEWIRE_SKIP_ROM);
}

bool onewire_power(int pin)
{
    // Make sure the bus is not being held low before driving it high, or we
    // may end up shorting ourselves out.
    if (!_onewire_wait_for_bus(pin, 10)) {
        return false;
    }
    ow_pin_pp(pin);
    return true;
}

void onewire_depower(int pin)
{
    ow_pin_od(pin);
}

void onewire_search_start(onewire_search_t *search)
{
    // reset the search state
    memset(search, 0, sizeof(*search));
}

void onewire_search_prefix(onewire_search_t *search, uint8_t family_code)
{
    search->rom_no[0] = family_code;
    for (uint8_t i = 1; i < 8; i++) {
        search->rom_no[i] = 0;
    }
    search->last_discrepancy = 64;
    search->last_device_found = false;
}

// Perform a search. If the next device has been successfully enumerated, its
// ROM address will be returned.  If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then ONEWIRE_NONE is returned.  Use onewire_search_start() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
//
onewire_addr_t onewire_search_next(onewire_search_t *search, int pin)
{
    uint8_t id_bit_number;
    uint8_t last_zero, search_result;
    int rom_byte_number;
    int8_t id_bit, cmp_id_bit;
    onewire_addr_t addr;
    unsigned char rom_byte_mask;
    bool search_direction;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    // if the last call was not the last one
    if (!search->last_device_found) {
        // 1-Wire reset
        if (!onewire_reset(pin) || !onewire_write(pin, ONEWIRE_SEARCH)) {
            // reset the search
            search->last_discrepancy = 0;
            search->last_device_found = false;
            return ONEWIRE_NONE;
        }

        // loop to do the search
        do {
            // read a bit and its complement
            id_bit = _onewire_read_bit(pin);
            cmp_id_bit = _onewire_read_bit(pin);

            // check for no devices on 1-wire
            if ((id_bit < 0) || (cmp_id_bit < 0)) {
                // Read error
                break;
            } else if ((id_bit == 1) && (cmp_id_bit == 1)) {
                break;
            } else {
                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit) {
                    search_direction = id_bit;  // bit write value for search
                } else {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < search->last_discrepancy) {
                        search_direction = ((search->rom_no[rom_byte_number] & rom_byte_mask) > 0);
                    } else {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == search->last_discrepancy);
                    }

                    // if 0 was picked then record its position in LastZero
                    if (!search_direction) {
                        last_zero = id_bit_number;
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction) {
                    search->rom_no[rom_byte_number] |= rom_byte_mask;
                } else {
                    search->rom_no[rom_byte_number] &= ~rom_byte_mask;
                }

                // serial number search direction write bit
                if (!_onewire_write_bit(pin, search_direction)) {
                    break;
                }

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0) {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        } while (rom_byte_number < 8);  // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65)) {
            // search successful so set last_discrepancy,last_device_found,search_result
            search->last_discrepancy = last_zero;

            // check for last device
            if (search->last_discrepancy == 0) {
                search->last_device_found = true;
            }

            search_result = 1;
        }
    }

    // if no device found, or the ROM code is corrupted, reset counters so next 'search' will be
    // like a first
    if (!search_result || !search->rom_no[0] ||
        onewire_crc8(search->rom_no, 7) != search->rom_no[7]) {
        search->last_discrepancy = 0;
        search->last_device_found = false;
        return ONEWIRE_NONE;
    }
    addr = 0;
    for (rom_byte_number = 7; rom_byte_number >= 0; rom_byte_number--) {
        addr = (addr << 8) | search->rom_no[rom_byte_number];
    }
    return addr;
}

// The 1-Wire CRC scheme is described in Maxim Application Note 27:
// "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
//

#if ONEWIRE_CRC8_TABLE
// This table comes from Dallas sample code where it is freely reusable,
// though Copyright (C) 2000 Dallas Semiconductor Corporation
static const uint8_t dscrc_table[] = {
      0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95,  1,227,189, 62, 96,130,220,
     35,125,159,193, 66, 28,254,160,225,191, 93,  3,128,222, 60, 98,
    190,224,  2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
     70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89,  7,
    219,133,103, 57,186,228,  6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135,  4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91,  5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
     17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
     50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73,  8, 86,180,234,105, 55,213,139,
     87,  9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53};

//
// Compute a Dallas Semiconductor 8 bit CRC. These show up in the ROM
// and the registers.
//
uint8_t onewire_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
        crc = dscrc_table[crc ^ *data++];
    }
    return crc;
}
#else
//
// Compute a Dallas Semiconductor 8 bit CRC directly.
// this is much slower, but much smaller, than the lookup table.
//
uint8_t onewire_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    while (len--) {
        uint8_t inbyte = *data++;
        for (int i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}
#endif

// Compute the 1-Wire CRC16 and compare it against the received CRC.
bool onewire_check_crc16(const uint8_t *input, size_t len, const uint8_t *inverted_crc, uint16_t crc_iv)
{
    uint16_t crc = ~onewire_crc16(input, len, crc_iv);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}

// Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
// the integrity of data received from many 1-Wire devices.  Note that the
// CRC computed here is *not* what you'll get from the 1-Wire network,
// for two reasons:
//   1) The CRC is transmitted bitwise inverted.
//   2) Depending on the endian-ness of your processor, the binary
//      representation of the two-byte return value may have a different
//      byte order than the two bytes you get from 1-Wire.
uint16_t onewire_crc16(const uint8_t *input, size_t len, uint16_t crc_iv)
{
    uint16_t crc = crc_iv;
    static const uint8_t oddparity[16] =
        { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    for (size_t i = 0; i < len; i++) {
        // Even though we're just copying a byte from the input,
        // we'll be doing 16-bit computation with it.
        uint16_t cdata = input[i];
        cdata = (cdata ^ crc) & 0xff;
        crc >>= 8;

        if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4]) {
            crc ^= 0xC001;
        }

        cdata <<= 6;
        crc ^= cdata;
        cdata <<= 1;
        crc ^= cdata;
    }
    return crc;
}
