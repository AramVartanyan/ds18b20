# Changelog

All notable changes to this component are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the
versions follow [Semantic Versioning](https://semver.org/).

## [Unreleased]

## [0.2.0] — 2026-11-08, tested on hardware

Review of both drivers against current ESP-IDF practice and the DS18B20 datasheet. Built with
ESP-IDF 5.5.1 for ESP32, ESP32-S3 and ESP32-C3 without warnings (also with `-Wextra -Werror`).
Tested on ESP32-S3 (ESP-IDF 5.5.4) in solar-heater-hap: two externally powered DS18B20 probes
(95 cm cable) on separate pins with the internal pull-ups, both converting at the same time —
stable readings, fast response to warming. The ESP8266_RTOS_SDK, ESP-IDF 4.x and esp-open-rtos
paths are kept but were not built.

### Added
- ESP-IDF 4.x / 5.x support for all ESP32 chips next to ESP8266_RTOS_SDK and esp-open-rtos:
  platform selection by `ESP_PLATFORM` / `CONFIG_IDF_TARGET_ESP8266`, `esp_rom_delay_us()`
  (ESP-IDF ≥ 4.3), spinlock critical sections (`portMUX_TYPE`) on ESP32 chips.
- `CMakeLists.txt`, `component.mk`, `Kconfig`.
- `CONFIG_ONEWIRE_INTERNAL_PULLUP` (default off): internal pull-up on the 1-Wire pins.
- Power mode detection with Read Power Supply [B4h], once per pin (`ds18b20_is_parasite()`):
  the strong pull-up is used only for parasite powered sensors.
- `ds18b20_wait()`: externally powered sensors are polled with read time slots until the
  conversion is done (datasheet) instead of always waiting 750 ms;
  `CONFIG_DS18B20_POLL_CONVERSION` turns it off (clones).
- `onewire_write_power()`: last bit and strong pull-up in one critical section (the datasheet
  asks for the pull-up within 10 µs after Convert T); `onewire_read_bit()`.
- `ds18b20_read_raw()` (1/16 °C for both families) and `ds18b20_raw_to_tenths()`.
- `DS18B20_RAW_POWER_ON`: `ds18b20_measure_and_read*()` measure once more when they read exactly
  +85 °C, the power-on value of a sensor that lost power for a moment.
- README with credits, wiring, datasheet compliance table; the original copyright notices
  (esp-open-rtos onewire: MIT, zeroday nodemcu.com, Dallas Semiconductor terms; esp-open-rtos
  ds18b20: BSD-3-Clause, SuperHouse Automation) in LICENSE.

### Changed
- 1-Wire timing per datasheet / Maxim AN126: write 1 low 6 µs (was 10 µs — with a weak pull-up the
  line had too little time to rise before the sensor samples at 15 µs), read sample at ~13 µs,
  write 0 low 60 µs.
- Interrupts are disabled only inside one time slot (≤ 70 µs); `ds18b20_measure()` no longer
  disables them for a whole byte.
- The GPIO is configured once per pin (`gpio_config()` logs every call on ESP-IDF); later only the
  direction changes between open drain and push-pull.
- `ds18b20_read_all()` and `ds18b20_read_single()` (deprecated) use the new functions.

### Fixed
- ESP32 chips: the pin was set to output-only open drain, where `gpio_get_level()` always reads 0;
  now input + output open drain.
- Reset: the driver went on as soon as the bus was high after the presence pulse; now it keeps
  the bus high for the full tRSTH ≥ 480 µs.
- `DS18B20_ANY` (Skip ROM) with a DS18B20 used the DS18S20 formula (the family byte of
  `DS18B20_ANY` is 0xFF): wrong temperature. Without an address the family is now taken from the
  configuration byte; DS1822 and other 12-bit parts no longer get the DS18S20 formula either.
- DS18S20 extended resolution subtracted 0.25 °C twice (also in esp-open-rtos).
- Undefined low bits at 9/10/11-bit resolution are cleared (datasheet).
- Read errors were ignored in `ds18b20_read_scratchpad()` (a failed byte was stored as 0xFF); a bus
  held low read all zeros, which pass the CRC check — both are errors now.
- `onewire_search_next()`: write errors were ignored; the ROM code CRC is checked.
- Deprecated `ds18b20_read_all()` / `ds18b20_read_single()`: negative temperatures were read as
  unsigned, errors returned 0 °C (now NaN).
- Removed the misdeclared `extern volatile long countdelay = 0;`, the unused busy loop
  `delay_100us()` and the global `gpio_enable()` (name clash risk).

## [0.1.0]

- esp-open-rtos `extras/onewire` and `extras/ds18b20` ported to ESP8266_RTOS_SDK 3.3.
