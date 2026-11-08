#ifndef DRIVER_DS18B20_H_
#define DRIVER_DS18B20_H_

#include "onewire.h"

#ifdef    __cplusplus
extern "C" {
#endif

/** @file ds18b20.h
 *
 *  Communicate with the DS18B20 family of one-wire temperature sensor ICs
 *  (DS18B20, DS1822 and other 12-bit parts; DS18S20 with its 9-bit format).
 *
 *  ESP-IDF 4.x / 5.x (all ESP32 chips), ESP8266_RTOS_SDK and esp-open-rtos. Based on
 *  esp-open-rtos extras/ds18b20; see LICENSE for the authors.
 *
 *  Power: an externally powered sensor (VDD pin) is the recommended way. The driver finds out
 *  with Read Power Supply [B4h] (once per pin) whether a device on the bus is parasite powered;
 *  then it applies the strong pull-up during the conversion, otherwise it polls the end of the
 *  conversion with read time slots (datasheet) instead of always waiting 750 ms.
 */

#define DS18B20_VERSION "0.2.0"

typedef onewire_addr_t ds18b20_addr_t;

/** An address value which can be used to indicate "any device on the bus" */
#define DS18B20_ANY ONEWIRE_NONE

/** Raw temperature (1/16 degC) of the power-on reset value, +85 degC (datasheet: 0550h). Read
 *  without a conversion after the sensor lost power for a moment; a real 85.0 degC reads the
 *  same, so ds18b20_measure_and_read*() measure once more when they get it. */
#define DS18B20_RAW_POWER_ON 0x0550

/** Find the addresses of all DS18B20 devices on the bus.
 *
 *  Scans the bus for all devices and places their addresses in the supplied
 *  array.  If there are more than `addr_count` devices on the bus, only the
 *  first `addr_count` are recorded.
 *
 *  @param pin         The GPIO pin connected to the DS18B20 bus
 *  @param addr_list   A pointer to an array of ds18b20_addr_t values.  This
 *                     will be populated with the addresses of the found
 *                     devices.
 *  @param addr_count  Number of slots in the `addr_list` array.  At most this
 *                     many addresses will be returned.
 *
 *  @returns The number of devices found.  Note that this may be less than,
 *  equal to, or more than `addr_count`, depending on how many DS18B20 devices
 *  are attached to the bus.
 */
int ds18b20_scan_devices(int pin, ds18b20_addr_t *addr_list, int addr_count);

/** Tell one or more sensors to perform a temperature measurement and
 *  conversion (CONVERT_T) operation.  This operation can take up to 750ms to
 *  complete.
 *
 *  Parasite power: the bus is driven high (strong pull-up) right after the command.
 *  External power: the bus stays free during the conversion.
 *
 *  If `wait=true`, ds18b20_wait() is called: it returns as soon as the conversion is done
 *  (external power) or after 750 ms (parasite power). If `wait=false`, it returns at once; the
 *  caller calls ds18b20_wait() later (e.g. after starting the conversion on several pins).
 *
 *  @param pin   The GPIO pin connected to the DS18B20 device
 *  @param addr  The 64-bit address of the device on the bus.  This can be set
 *               to ::DS18B20_ANY to send the command to all devices on the bus
 *               at the same time.
 *  @param wait  Whether to wait for the necessary 750ms for the DS18B20 to
 *               finish performing the conversion before returning to the
 *               caller (You will normally want to do this).
 *
 *  @returns `true` if the command was successfully issued, or `false` on error.
 */
bool ds18b20_measure(int pin, ds18b20_addr_t addr, bool wait);

/** Wait for the conversion started by ds18b20_measure(pin, addr, false) to end.
 *
 *  Externally powered devices: read time slots every 10 ms until all addressed devices answer 1
 *  (conversion done), at most 1 s. Parasite power (or polling disabled in Kconfig): waits the
 *  full 750 ms, then releases the strong pull-up.
 *
 *  @returns `true` when the conversion is done, `false` on error or timeout.
 */
bool ds18b20_wait(int pin);

/** Whether at least one device on the bus is parasite powered (Read Power Supply [B4h]).
 *
 *  The answer is cached per pin and read again after a failed reset. When the bus does not answer,
 *  `true` is returned: the strong pull-up is the safe choice.
 */
bool ds18b20_is_parasite(int pin);

/** Read the value from the last CONVERT_T operation as raw temperature in 1/16 degC.
 *
 *  Both formats are returned in 1/16 degC: DS18S20 with its extended resolution (COUNT_REMAIN),
 *  12-bit parts with the bits that the configured resolution leaves undefined cleared.
 *
 *  @returns `true` on success; `false` when the device does not answer or the CRC is wrong.
 */
bool ds18b20_read_raw(int pin, ds18b20_addr_t addr, int16_t *raw);

/** Raw temperature (1/16 degC) to tenths of a degree, rounded (e.g. 0x0191 = 25.0625 -> 251). */
int16_t ds18b20_raw_to_tenths(int16_t raw);

/** Read the value from the last CONVERT_T operation.
 *
 *  This should be called after ds18b20_measure() to fetch the result of the
 *  temperature measurement.
 *
 *  @param pin     The GPIO pin connected to the DS18B20 device
 *  @param addr    The 64-bit address of the device to read.  This can be set
 *                 to ::DS18B20_ANY to read any device on the bus (but note
 *                 that this will only work if there is exactly one device
 *                 connected, or they will corrupt each others' transmissions)
 *
 *  @returns The temperature in degrees Celsius, or NaN if there was an error.
 */
float ds18b20_read_temperature(int pin, ds18b20_addr_t addr);

/** Read the value from the last CONVERT_T operation for multiple devices.
 *
 *  This should be called after ds18b20_measure() to fetch the result of the
 *  temperature measurement.
 *
 *  @param pin         The GPIO pin connected to the DS18B20 bus
 *  @param addr_list   A list of addresses for devices to read.
 *  @param addr_count  The number of entries in `addr_list`.
 *  @param result_list An array of floats to hold the returned temperature
 *                     values.  It should have at least `addr_count` entries.
 *
 *  @returns `true` if all temperatures were fetched successfully, or `false`
 *  if one or more had errors (the temperature for erroring devices will be
 *  returned as NaN).
 */
bool ds18b20_read_temp_multi(int pin, ds18b20_addr_t *addr_list, int addr_count, float *result_list);

/** Perform a ds18b20_measure() followed by ds18b20_read_temperature()
 *
 *  A result of exactly +85 degC (the power-on value) is measured once more.
 *
 *  @param pin     The GPIO pin connected to the DS18B20 device
 *  @param addr    The 64-bit address of the device to read.  This can be set
 *                 to ::DS18B20_ANY to read any device on the bus (but note
 *                 that this will only work if there is exactly one device
 *                 connected, or they will corrupt each others' transmissions)
 *
 *  @returns The temperature in degrees Celsius, or NaN if there was an error.
 */
float ds18b20_measure_and_read(int pin, ds18b20_addr_t addr);

/** Perform a ds18b20_measure() followed by ds18b20_read_temp_multi()
 *
 *  If a device returns exactly +85 degC (the power-on value), all are measured once more.
 *
 *  @param pin         The GPIO pin connected to the DS18B20 bus
 *  @param addr_list   A list of addresses for devices to read.
 *  @param addr_count  The number of entries in `addr_list`.
 *  @param result_list An array of floats to hold the returned temperature
 *                     values.  It should have at least `addr_count` entries.
 *
 *  @returns `true` if all temperatures were fetched successfully, or `false`
 *  if one or more had errors (the temperature for erroring devices will be
 *  returned as NaN).
 */
bool ds18b20_measure_and_read_multi(int pin, ds18b20_addr_t *addr_list, int addr_count, float *result_list);

/** Read the scratchpad data for a particular DS18B20 device.
 *
 *  This is not generally necessary to do directly.  It is done automatically
 *  as part of ds18b20_read_temperature().
 *
 *  @param pin     The GPIO pin connected to the DS18B20 device
 *  @param addr    The 64-bit address of the device to read.  This can be set
 *                 to ::DS18B20_ANY to read any device on the bus (but note
 *                 that this will only work if there is exactly one device
 *                 connected, or they will corrupt each others' transmissions)
 *  @param buffer  An 8-byte buffer to hold the read data.
 *
 *  @returns `true` if the data was read successfully, or `false` on error.
 */
bool ds18b20_read_scratchpad(int pin, ds18b20_addr_t addr, uint8_t *buffer);

// The following are obsolete/deprecated APIs (kept for older code; errors give NaN)

typedef struct {
    uint8_t id;
    float value;
} ds_sensor_t;

// Scan all ds18b20 sensors on bus and return its amount.
// Result are saved in array of ds_sensor_t structure.
uint8_t ds18b20_read_all(uint8_t pin, ds_sensor_t *result);

// This method is just to demonstrate how to read
// temperature from single dallas chip.
float ds18b20_read_single(uint8_t pin);

#ifdef    __cplusplus
}
#endif

#endif  /* DRIVER_DS18B20_H_ */
