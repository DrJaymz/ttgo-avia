# Changes

2026-03-13
- Implement tacho input on `INTERRUPT_PIN` using ISR pulse-period timing (`esp_timer_get_time`) for accurate RPM estimation.
- Add tach signal validation (minimum pulse interval + timeout to drop RPM to zero when pulses stop).
- Add light RPM post-conversion smoothing (`kTauRpmSec = 0.4s`) for quick response with reduced flicker.
- Show RPM on the sender screen alongside CHT.

2026-03-07
- Show the startup current zero calibration value (`ampOffset`) next to the amp reading to help choose a fixed zero point.
- Add per-channel low-pass smoothing on converted values:
- Fuel quantity uses 20s damping; battery uses 4s; amp uses 5s; fuel pressure, oil temp, oil pressure, and CHT use 5s.
- Guard CHT processing against invalid/non-finite values so occasional bogus sensor outputs are clamped safely.

2026-02-23
- Add root `README.md` with an ESP32 connection summary for ADC sensors, thermocouple, buttons, and TFT wiring.
- Document currently defined-but-unused GPIO definitions to avoid wiring ambiguity.
- Add documentation note that `ADC2` cannot be used for reliable analog reads while Wi-Fi/ESP-NOW is active.
- Add I2C guidance with recommended pins (`GPIO21`/`GPIO22`) and pull-up requirement.

2026-02-05
- Add AGENTS.md to document sender behavior, telemetry contract, and files of interest.
- Add CHANGES.md for tracking future sender firmware updates.
