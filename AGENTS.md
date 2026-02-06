# Project: ttgo-avia (ESP-NOW Sender)

## Overview
- ESP32/TTGO sender for the Avia engine monitoring system.
- Reads analog sensors + thermocouple, filters data, and sends telemetry via ESP-NOW.
- Optional simulation mode for bench testing.

## Key Behaviors
- Timer-driven sampling (`UPDATE_RATE_USEC`) with screen updates and ESP-NOW frame sends.
- Median filtering on ADC channels; thermocouple read throttled.
- Simulation mode (`SIMULATE`) injects fixed ADC values with a slow varying multiplier.

## Files of Interest
- `src/main.cpp`: sampling loop, filtering, UI text, and ESP-NOW send.
- `src/espNow.h`: ESPNOW peer MAC and `SensorData` payload definition.
- `src/global.h`: pin map, ADC channels, constants, and globals.
- `platformio.ini`: build environment for the sender board.

## Telemetry Contract
- `SensorData` layout must match the display receiver’s expected fields.
- If fields, units, or ordering change, update the display firmware accordingly.
- Receiver project: `../Avia Display` (keep in sync when changing telemetry).

## Notes for Future Changes
- Keep sensor calibration math and filter settings aligned with real hardware.
- If simulation values change, update any display-side assumptions or demos.
- Document change sets in `CHANGES.md`.
