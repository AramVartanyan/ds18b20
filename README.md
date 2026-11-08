# ds18b20

1-Wire bus master and driver for the DS18B20 family of temperature sensors (DS18B20, DS1822 and
other 12-bit parts, DS18S20) for ESP chips.

> **Credits.** This library is based on the `onewire` and `ds18b20` drivers of
> [esp-open-rtos](https://github.com/SuperHouse/esp-open-rtos) (`extras/onewire`, `extras/ds18b20`).
> The 1-Wire part goes back to the Arduino [OneWire](https://github.com/PaulStoffregen/OneWire)
> library by Jim Studt, Paul Stoffregen and many contributors; the search algorithm and the CRC
> come from Dallas Semiconductor / Maxim. The original copyright notices are kept in
> [LICENSE](LICENSE).

**Version:** 0.2.0 — tested on hardware: ESP32-S3 (ESP-IDF 5.5.4), two externally powered DS18B20 on
separate pins with the internal pull-ups, in solar-heater-hap. See [Changelog.md](Changelog.md).

## Platforms

- **ESP-IDF 4.x / 5.x**, all ESP32 chips (built with ESP-IDF 5.5.1 for ESP32, ESP32-S3, ESP32-C3).
- **ESP8266_RTOS_SDK** 3.x (CMake and make) — the code path is kept, not built here (no SDK in
  the build environment); test before use.
- **esp-open-rtos** (the original platform) — kept the same way, not built here.

Platform selection like in esp_tm1637: `ESP_PLATFORM` (ESP-IDF and ESP8266_RTOS_SDK) and
`CONFIG_IDF_TARGET_ESP8266`; delays with `esp_rom_delay_us()` (ESP-IDF ≥ 4.3) or `ets_delay_us()`.

## Wiring

- One GPIO per bus; several sensors may share a bus (each has its 64-bit ROM code).
- **Pull-up resistor 4.7 kΩ** from DATA to the MCU logic level (3.3 V for ESP32 chips — their
  pins are not 5 V tolerant). The sensor itself may be powered from 5 V (it accepts a high level
  from 2.2 V).
- **External power (VDD pin) is recommended**; parasite power works too (the driver finds out and
  applies the strong pull-up), but the datasheet does not recommend it above +100 °C.
- The internal pull-up (`CONFIG_ONEWIRE_INTERNAL_PULLUP`, ~45 kΩ on ESP32 chips) is enough only
  for short wires (a development setup).

## Datasheet compliance

Timing of the bit-banged 1-Wire master (DS18B20 datasheet AC characteristics, standard values of
Maxim application note 126):

| Item | Datasheet | Driver |
|---|---|---|
| Reset low (tRSTL) | ≥ 480 µs | 480 µs |
| Presence sample | 15 – 60 µs high, then 60 – 240 µs low | at 70 µs |
| Reset high (tRSTH) | ≥ 480 µs | 70 + 410 µs, then the bus must be high |
| Write 1 low (tLOW1) | 1 – 15 µs | 6 µs (a weak pull-up still has time to rise before the sensor samples at 15 µs) |
| Write 0 low (tLOW0) | 60 – 120 µs | 60 µs |
| Time slot (tSLOT) | 60 – 120 µs | 70 µs |
| Recovery (tREC) | ≥ 1 µs | 10 µs after write 0, waits for the bus to rise |
| Read: master low | ≥ 1 µs, as short as possible | 3 µs |
| Read: sample (tRDV) | before 15 µs | at ~13 µs |
| Strong pull-up (parasite) | within 10 µs after Convert T | at the end of the last bit, same critical section |
| Conversion (tCONV, 12 bits) | ≤ 750 ms | parasite: 750 ms; external: read time slots until done |
| Power-on value | +85 °C (0550h) | measured once more by `ds18b20_measure_and_read*()` |
| Undefined bits at 9 – 11 bits | bits 2..0 / 1..0 / 0 | cleared |

Interrupts are disabled only inside one time slot (≤ 70 µs), never for a whole byte, so Wi-Fi and
other tasks are not held up.

## API (summary)

```c
#include "ds18b20.h"

/* two sensors, each on its own pin: both convert at the same time */
ds18b20_measure(PIN_A, DS18B20_ANY, false);
ds18b20_measure(PIN_B, DS18B20_ANY, false);
ds18b20_wait(PIN_A);
ds18b20_wait(PIN_B);

int16_t raw;                                    /* 1/16 degC */
if (ds18b20_read_raw(PIN_A, DS18B20_ANY, &raw)) {
    int16_t t = ds18b20_raw_to_tenths(raw);     /* 251 = 25.1 degC */
}

float t = ds18b20_measure_and_read(PIN_A, DS18B20_ANY);   /* NaN on error */
```

`DS18B20_ANY` (Skip ROM) works only with one sensor on the bus; with several, use the addresses
from `ds18b20_scan_devices()`. The original API (`ds18b20_measure`, `ds18b20_read_temperature`,
`ds18b20_read_temp_multi`, `ds18b20_scan_devices`, the `onewire_*` functions...) is unchanged.

A bus (pin) must be used from one task at a time.

## Configuration (Kconfig, "DS18B20 / 1-Wire")

| Option | Default | |
|---|---|---|
| `CONFIG_ONEWIRE_INTERNAL_PULLUP` | n | internal pull-up on the 1-Wire pins |
| `CONFIG_DS18B20_POLL_CONVERSION` | y | poll the end of the conversion (external power); off → always 750 ms (for clones that do not support it) |

Debug output: define `DS18B20_DEBUG` for the component.

## Alternatives considered

Espressif's [`onewire_bus`](https://components.espressif.com/components/espressif/onewire_bus) and
[`ds18b20`](https://components.espressif.com/components/espressif/ds18b20) components use the RMT
peripheral (hardware timing, no interrupts disabled), but need ESP-IDF 5.x, take RMT channels and
do not run on ESP8266. This library keeps one code base for ESP8266_RTOS_SDK, ESP-IDF 4.x and 5.x.
