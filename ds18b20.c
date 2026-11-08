/*
 * DS18B20 family 1-Wire temperature sensors.
 *
 * Origin: esp-open-rtos extras/ds18b20 (BSD-3-Clause, (c) 2015 SuperHouse Automation Pty Ltd and
 * contributors, see LICENSE). Updated for ESP-IDF 4.x / 5.x and ESP8266_RTOS_SDK, checked against
 * the DS18B20 datasheet (Maxim 19-7487) by Aram Vartanyan, 2026.
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#else
#include "FreeRTOS.h"
#include "task.h"
#endif

#include "ds18b20.h"

#define DS18B20_WRITE_SCRATCHPAD 0x4E
#define DS18B20_READ_SCRATCHPAD  0xBE
#define DS18B20_COPY_SCRATCHPAD  0x48
#define DS18B20_READ_EEPROM      0xB8
#define DS18B20_READ_PWRSUPPLY   0xB4
#define DS18B20_SEARCHROM        0xF0
#define DS18B20_SKIP_ROM         0xCC
#define DS18B20_READROM          0x33
#define DS18B20_MATCHROM         0x55
#define DS18B20_ALARMSEARCH      0xEC
#define DS18B20_CONVERT_T        0x44

#define DS18B20_FAMILY_ID 0x28
#define DS18S20_FAMILY_ID 0x10

#define T_CONV_MS        750    /* tCONV at 12 bits (datasheet max) */
#define T_CONV_POLL_MS   10
#define T_CONV_LIMIT_MS  1000   /* polling gives up after this */

#ifndef CONFIG_DS18B20_POLL_CONVERSION
#define CONFIG_DS18B20_POLL_CONVERSION 1
#endif

#define os_sleep_ms(x) vTaskDelay(((x) + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

#ifdef DS18B20_DEBUG
#define debug(fmt, ...) printf("%s" fmt "\n", "DS18B20: ", ## __VA_ARGS__);
#else
#define debug(fmt, ...)
#endif

/* Power mode per pin: read once with Read Power Supply, again after a failed reset */
static uint64_t s_power_known;
static uint64_t s_parasite;

static bool address(int pin, ds18b20_addr_t addr)
{
    return addr == DS18B20_ANY ? onewire_skip_rom(pin) : onewire_select(pin, addr);
}

static bool reset(int pin)
{
    if (onewire_reset(pin)) {
        return true;
    }
    s_power_known &= ~(1ULL << pin);
    return false;
}

bool ds18b20_is_parasite(int pin)
{
    uint64_t bit = 1ULL << pin;
    if (s_power_known & bit) {
        return (s_parasite & bit) != 0;
    }
    // Skip ROM + Read Power Supply + read time slot: a parasite powered device pulls it low
    if (!reset(pin) || !onewire_skip_rom(pin) || !onewire_write(pin, DS18B20_READ_PWRSUPPLY)) {
        return true;
    }
    int b = onewire_read_bit(pin);
    if (b < 0) {
        return true;
    }
    s_power_known |= bit;
    if (b) {
        s_parasite &= ~bit;
    } else {
        s_parasite |= bit;
    }
    debug("pin %d: %s power", pin, b ? "external" : "parasite");
    return b == 0;
}

bool ds18b20_measure(int pin, ds18b20_addr_t addr, bool wait)
{
    bool parasite = ds18b20_is_parasite(pin);

    if (!reset(pin) || !address(pin, addr)) {
        return false;
    }
    if (parasite) {
        // strong pull-up within 10 us after the command (datasheet), held during tCONV
        if (!onewire_write_power(pin, DS18B20_CONVERT_T)) {
            return false;
        }
    } else if (!onewire_write(pin, DS18B20_CONVERT_T)) {
        return false;
    }
    return wait ? ds18b20_wait(pin) : true;
}

bool ds18b20_wait(int pin)
{
    uint64_t bit = 1ULL << pin;
    bool parasite = !(s_power_known & bit) || (s_parasite & bit);

    if (parasite || !CONFIG_DS18B20_POLL_CONVERSION) {
        os_sleep_ms(T_CONV_MS);
        if (parasite) {
            onewire_depower(pin);
        }
        return true;
    }
    // external power: the devices answer read time slots with 0 while converting, 1 when done
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        int b = onewire_read_bit(pin);
        if (b < 0) {
            return false;
        }
        if (b) {
            return true;
        }
        if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > T_CONV_LIMIT_MS) {
            debug("pin %d: conversion did not end", pin);
            return false;
        }
        os_sleep_ms(T_CONV_POLL_MS);
    }
}

bool ds18b20_read_scratchpad(int pin, ds18b20_addr_t addr, uint8_t *buffer)
{
    uint8_t sp[9];

    if (!reset(pin) || !address(pin, addr) || !onewire_write(pin, DS18B20_READ_SCRATCHPAD)) {
        return false;
    }
    if (!onewire_read_bytes(pin, sp, sizeof(sp))) {
        return false;
    }
    // a bus held low reads all zeros, and the CRC of zeros is zero too
    bool all_zero = true;
    for (int i = 0; i < 9; i++) {
        if (sp[i]) {
            all_zero = false;
            break;
        }
    }
    if (all_zero) {
        debug("scratchpad all zero (bus held low?)");
        return false;
    }
    uint8_t expected_crc = onewire_crc8(sp, 8);
    if (sp[8] != expected_crc) {
        debug("CRC check failed reading scratchpad: %02x %02x %02x %02x %02x %02x %02x %02x : %02x (expected %02x)",
              sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], sp[7], sp[8], expected_crc);
        return false;
    }
    memcpy(buffer, sp, 8);
    return true;
}

/* DS18S20 (9 bits + COUNT_REMAIN) or a 12-bit part (DS18B20, DS1822, ...). Without an address,
 * the configuration byte tells: DS18B20 "0 R1 R0 1 1 1 1 1", DS18S20 reserved 0xFF. */
static bool is_ds18s20(ds18b20_addr_t addr, const uint8_t *sp)
{
    if (addr != DS18B20_ANY) {
        return (uint8_t)addr == DS18S20_FAMILY_ID;
    }
    return sp[4] == 0xFF;
}

bool ds18b20_read_raw(int pin, ds18b20_addr_t addr, int16_t *raw)
{
    uint8_t sp[8];

    if (!ds18b20_read_scratchpad(pin, addr, sp)) {
        return false;
    }
    int16_t t = (int16_t)((uint16_t)sp[1] << 8 | sp[0]);
    if (is_ds18s20(addr, sp)) {
        // extended resolution (DS18S20 datasheet):
        // T = TEMP_READ - 0.25 + (COUNT_PER_C - COUNT_REMAIN) / COUNT_PER_C, COUNT_PER_C = 16
        t = (int16_t)((t & ~1) * 8 - 4 + (16 - sp[6]));
    } else {
        // at 11/10/9 bits the lowest 1/2/3 bits are undefined (datasheet)
        static const int16_t mask[4] = { ~7, ~3, ~1, ~0 };
        t &= mask[(sp[4] >> 5) & 3];
    }
    *raw = t;
    return true;
}

int16_t ds18b20_raw_to_tenths(int16_t raw)
{
    int32_t v = (int32_t)raw * 10;
    return (int16_t)((v + (v >= 0 ? 8 : -8)) / 16);
}

float ds18b20_read_temperature(int pin, ds18b20_addr_t addr)
{
    int16_t raw;
    if (!ds18b20_read_raw(pin, addr, &raw)) {
        return NAN;
    }
    return raw / 16.0f;
}

float ds18b20_measure_and_read(int pin, ds18b20_addr_t addr)
{
    int16_t raw = 0;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!ds18b20_measure(pin, addr, true) || !ds18b20_read_raw(pin, addr, &raw)) {
            return NAN;
        }
        if (raw != DS18B20_RAW_POWER_ON) {
            break;
        }
        debug("pin %d: 85 degC (power-on value?), measuring again", pin);
    }
    return raw / 16.0f;
}

bool ds18b20_measure_and_read_multi(int pin, ds18b20_addr_t *addr_list, int addr_count, float *result_list)
{
    bool result = false;
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!ds18b20_measure(pin, DS18B20_ANY, true)) {
            for (int i = 0; i < addr_count; i++) {
                result_list[i] = NAN;
            }
            return false;
        }
        result = ds18b20_read_temp_multi(pin, addr_list, addr_count, result_list);
        bool power_on = false;
        for (int i = 0; i < addr_count; i++) {
            if (result_list[i] == DS18B20_RAW_POWER_ON / 16.0f) {
                power_on = true;
            }
        }
        if (!power_on) {
            break;
        }
    }
    return result;
}

int ds18b20_scan_devices(int pin, ds18b20_addr_t *addr_list, int addr_count)
{
    onewire_search_t search;
    onewire_addr_t addr;
    int found = 0;

    onewire_search_start(&search);
    while ((addr = onewire_search_next(&search, pin)) != ONEWIRE_NONE) {   // ROM CRC checked
        uint8_t family_id = (uint8_t)addr;
        if (family_id == DS18B20_FAMILY_ID || family_id == DS18S20_FAMILY_ID) {
            if (found < addr_count) {
                addr_list[found] = addr;
            }
            found++;
        }
    }
    return found;
}

bool ds18b20_read_temp_multi(int pin, ds18b20_addr_t *addr_list, int addr_count, float *result_list)
{
    bool result = true;

    for (int i = 0; i < addr_count; i++) {
        result_list[i] = ds18b20_read_temperature(pin, addr_list[i]);
        if (isnan(result_list[i])) {
            result = false;
        }
    }
    return result;
}

/* ------------------------------------------------------- deprecated APIs */

uint8_t ds18b20_read_all(uint8_t pin, ds_sensor_t *result)
{
    onewire_addr_t addr;
    onewire_search_t search;
    uint8_t sensor_id = 0;

    onewire_search_start(&search);
    while ((addr = onewire_search_next(&search, pin)) != ONEWIRE_NONE) {
        result[sensor_id].id = sensor_id;
        result[sensor_id].value = ds18b20_measure_and_read(pin, addr);
        sensor_id++;
    }
    return sensor_id;
}

float ds18b20_read_single(uint8_t pin)
{
    return ds18b20_measure_and_read(pin, DS18B20_ANY);
}
