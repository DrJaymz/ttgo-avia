#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <Arduino.h>
#include <TelnetSpy.h>
#include <TFT_eSPI.h>

// intellisense workaround
//  _VOID      _EXFUN(tzset,	(_VOID));
//  int	_EXFUN(setenv,(const char *__string, const char *__value, int __overwrite));

#define ADC_EN 14 // ADC_EN is the ADC detection enable port

// Brown  Signal Ground
#define ADC_BAT 36 // RED    Battery Voltage
#define ADC_CH1 37 // Orange CH1 Fuel Qty
#define ADC_CH2 38 // Yellow CH2 Fuel Pressure
#define ADC_CH3 33 // Green  CH3 Oil Temp
#define ADC_CH4 32 // Blue   CH4 Oil Pressure
#define ADC_AMP 39 //       CH5 Current Sensor

#define THERM_SCK 12
#define THERM_CS 13
#define THERM_SO 15

#define BUTTON_1 35
#define BUTTON_2 0
#define TFT_BACKLIGHT_ON 1
#define HOSTNAME "AVIA"

#define INTERRUPT_PIN 26
#define DEBUG 1
#define CHT1_COMPENSATION -15

// Firmware version history (inferred from repo history + CHANGES.md):
// 1.0.0 - Initial tagged/recorded repo version ("version 1.0" commit).
// 1.0.1 - 2026-02-05 documentation tracking added (AGENTS.md, CHANGES.md).
// 1.0.2 - 2026-02-23 README and wiring/docs update.
// 1.1.0 - 2026-03-07 sensor smoothing and amp zero-cal display update.
// 1.2.0 - 2026-03-13 tach input / RPM feature added.
// 1.2.1 - 2026-03-16 tach reacquire and stale-period fix.
// 2.0.0 - 2026-04-23 telemetry contract change (SensorData payload updated).
// 2.0.1 - 2026-05-01 ESP-NOW transmit rate reduced from 10 Hz to 5 Hz.
constexpr uint8_t FW_VERSION_MAJOR = 2;
constexpr uint8_t FW_VERSION_MINOR = 0;
constexpr uint8_t FW_VERSION_PATCH = 1;
constexpr char FW_VERSION[] = "2.0.1";

// ESP-NOW peer MAC (receiver). Update this to match the display unit.
extern uint8_t kRemotePeerMacAddress[6];

extern TelnetSpy debug;
extern TFT_eSPI tft;

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();

#define AA_FONT_SMALL "NotoSansBold15"
#define AA_FONT_LARGE "NotoSansBold36"

// Font files are stored in SPIFFS, so load the library
#include <FS.h>

#endif
