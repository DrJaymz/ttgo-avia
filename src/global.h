#ifndef GLOBAL_VARS_H
#define GLOBAL_VARS_H

#include <Arduino.h>
#include <TelnetSpy.h>
#include <TFT_eSPI.h>

//intellisense workaround 
// _VOID      _EXFUN(tzset,	(_VOID));
// int	_EXFUN(setenv,(const char *__string, const char *__value, int __overwrite));

#define ADC_EN 14 // ADC_EN is the ADC detection enable port

#define ADC_BAT 36
#define ADC_CH1 37
#define ADC_CH2 38
#define ADC_CH3 33
#define ADC_CH4 32

#define BUTTON_1 35
#define BUTTON_2 0
#define TFT_BACKLIGHT_ON 1
#define HOSTNAME "AVIA"

#define INTERRUPT_PIN 26
#define DEBUG 1

extern TelnetSpy debug;
extern TFT_eSPI tft;

#define AA_FONT_SMALL "NotoSansBold15"
#define AA_FONT_LARGE "NotoSansBold36"

// Font files are stored in SPIFFS, so load the library
#include <FS.h>

#endif