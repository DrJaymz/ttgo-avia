#ifndef LCD_H /* include guards */
#define LCD_H

#include <ArduinoOTA.h>

void TFT_printLine(String line, bool clearScreen = false);
void printScreenLine();
// void TFT_wake();
// void TFT_sleep();

#endif /* LCD_H */