# Changes

2026-05-01 - v2.0.1
- Halve ESP-NOW telemetry transmit rate from 10 Hz to 5 Hz by sending every second screen/data frame while keeping the 100 Hz sampling loop and existing smoothing time constants unchanged.

2026-04-23 - v2.0.0
- Add explicit `rpm` and `rpmError` fields to the ESP-NOW `SensorData` payload on sender and display; this is a telemetry contract change and both firmwares must be updated together.
- Validate RPM to `0..3000` before transmit, flag out-of-range/non-finite values as a fault, and align the sender tach pulse filter to the 3000 RPM maximum.
- Tighten sender simulation values to healthy operating bands and explicitly clear simulated sensor fault flags to avoid flickering `0/1` error indicators during bench testing.
- Split RPM simulation from the general sensor simulation so bench-simulated analog channels can run while the tach input remains live.

2026-03-16 - v1.2.1
- Fix tach RPM reacquire after long zero-RPM idle: first pulse after timeout is now treated as a sync edge, and RPM period measurement resumes on the next pulse.
- Prevent stale long period values from blocking valid tach updates when restarting from 0 RPM.

2026-03-13 - v1.2.0
- Implement tacho input on `INTERRUPT_PIN` using ISR pulse-period timing (`esp_timer_get_time`) for accurate RPM estimation.
- Add tach signal validation (minimum pulse interval + timeout to drop RPM to zero when pulses stop).
- Add light RPM post-conversion smoothing (`kTauRpmSec = 0.4s`) for quick response with reduced flicker.
- Show RPM on the sender screen alongside CHT.

2026-03-07 - v1.1.0
- Show the startup current zero calibration value (`ampOffset`) next to the amp reading to help choose a fixed zero point.
- Add per-channel low-pass smoothing on converted values:
- Fuel quantity uses 20s damping; battery uses 4s; amp uses 5s; fuel pressure, oil temp, oil pressure, and CHT use 5s.
- Guard CHT processing against invalid/non-finite values so occasional bogus sensor outputs are clamped safely.

2026-02-23 - v1.0.2
- Add root `README.md` with an ESP32 connection summary for ADC sensors, thermocouple, buttons, and TFT wiring.
- Document currently defined-but-unused GPIO definitions to avoid wiring ambiguity.
- Add documentation note that `ADC2` cannot be used for reliable analog reads while Wi-Fi/ESP-NOW is active.
- Add I2C guidance with recommended pins (`GPIO21`/`GPIO22`) and pull-up requirement.

2026-02-05 - v1.0.1
- Add AGENTS.md to document sender behavior, telemetry contract, and files of interest.
- Add CHANGES.md for tracking future sender firmware updates.
