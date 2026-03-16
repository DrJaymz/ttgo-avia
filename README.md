# ttgo-avia (ESP-NOW Sender)

ESP32/TTGO sender firmware for Avia engine telemetry.

## ESP32 Connection Summary

### Analog Sensor Inputs (ADC)
| ESP32 GPIO | Define | Signal |
|---|---|---|
| 36 | `ADC_BAT` | Battery voltage (RED) |
| 37 | `ADC_CH1` | Fuel quantity (Orange) |
| 38 | `ADC_CH2` | Fuel pressure (Yellow) |
| 33 | `ADC_CH3` | Oil temperature (Green) |
| 32 | `ADC_CH4` | Oil pressure (Blue) |
| 39 | `ADC_AMP` | Current sensor (CH5) |
| GND | n/a | Sensor ground (Brown) |

### Thermocouple Interface (MAX6675)
| ESP32 GPIO | Define | Signal |
|---|---|---|
| 12 | `THERM_SCK` | SPI clock |
| 13 | `THERM_CS` | Chip select |
| 15 | `THERM_SO` | Data out (MISO from MAX6675) |

### Buttons / Wake
| ESP32 GPIO | Define | Use |
|---|---|---|
| 35 | `BUTTON_1` | Button input, also deep-sleep wake (`esp_sleep_enable_ext0_wakeup`) |
| 0 | `BUTTON_2` | Button input |

### TFT Display (ST7789 via `TFT_eSPI` build flags)
| ESP32 GPIO | Build Flag | Signal |
|---|---|---|
| 19 | `-DTFT_MOSI=19` | SPI MOSI |
| 18 | `-DTFT_SCLK=18` | SPI SCLK |
| -1 (not used) | `-DTFT_MISO=-1` | SPI MISO |
| 5 | `-DTFT_CS=5` | Chip select |
| 16 | `-DTFT_DC=16` | Data/command |
| 23 | `-DTFT_RST=23` | Reset |
| 4 | `-DTFT_BL=4` | Backlight control |

## Defined But Not Currently Used In Logic
- `ADC_EN` on GPIO 14 is defined in `src/global.h` but not toggled in `src/main.cpp`.
- `INTERRUPT_PIN` on GPIO 26 is defined but not used.
- `DAC_CH1` on GPIO 25 is defined in `src/main.cpp` but not used.

## ADC2 Limitation (Important)
- This project uses Wi-Fi/ESP-NOW, so `ADC2` analog inputs are not reliable/usable while radio is active.
- Do not plan new analog sensors on `ADC2` pins.
- Common candidate GPIOs like `25`, `26`, and `27` are fine for digital I/O (for example tach pulse input), but not for analog acquisition in this firmware mode.

## I2C Availability
- There are enough free GPIOs for I2C.
- Recommended mapping on ESP32: `GPIO21` as `SDA` and `GPIO22` as `SCL`.
- I2C needs pull-ups to `3.3V` (typically `4.7k`), unless your module already provides them.
- Using `GPIO21/22` for I2C keeps `GPIO25/26/27/17` available for other digital signals (for example tach input).

## Source References
- `src/global.h`
- `src/main.cpp`
- `platformio.ini`
