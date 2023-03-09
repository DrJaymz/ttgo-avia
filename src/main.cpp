#include <Arduino.h>
#include <Ticker.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "esp_adc_cal.h"
#include <FS.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include "WiFiManager.h" //https://github.com/tzapu/WiFiManager

#include "Filter.h"
#include "global.h"
#include "ota.h"
#include "lcd.h"
#include "Button2.h"
#include "espNow.h"

#include <esp32-hal-timer.h>
#include <driver/dac.h>
#include <math.h>

// other crap
char string[16];
TelnetSpy debug;

#define DAC_CH1 25
#define ADC_RES 12
#define ADC_FILTER 1 // new value weight out of 100

ExponentialFilter<float> filterBat(ADC_FILTER, 0.0);
ExponentialFilter<float> filterCh1(ADC_FILTER, 0.0);
ExponentialFilter<float> filterCh2(ADC_FILTER, 0.0);
ExponentialFilter<float> filterCh3(ADC_FILTER, 0.0);
ExponentialFilter<float> filterCh4(ADC_FILTER, 0.0);
ESPNowTransmitter espNow(remotePeerMacAddress);

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

  esp_light_sleep_start();
}

// Buttons ***************************************************************************************************

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

// int btnCick = false;
void button_init()
{
  btn1.setLongClickHandler([](Button2 &b)
                           {
        //btnCick = false;
        int r = digitalRead(TFT_BL);
        TFT_printLine("Sleep...",true);
        espDelay(1000);
        tft.fillScreen(TFT_BLACK);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);  
         delay(100);
        tft.writecommand(TFT_SLPIN);    //tft sleep
        delay(100);

        //rtc_gpio_isolate(GPIO_NUM_35);

        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
        delay(200);
        esp_deep_sleep_start(); });

  btn1.setPressedHandler([](Button2 &b) {

  });

  btn2.setPressedHandler([](Button2 &b)
                         {
                            if (true)
                           {} });
}

// Buttons ***************************************************************************************************

// volatile byte interruptCounter = 0;
volatile int timerElapsed = 0;
volatile bool screenUpdate = false;
volatile bool adcTimer = false;
int vref = 1100;
TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

void ICACHE_RAM_ATTR handleInterrupt()
{
}

void setupWifi()
{
  byte mac[6];
  WiFi.macAddress(mac);
  char buf[30];
  sprintf(buf, "%s_%02X%02X", HOSTNAME, mac[4], mac[5]);
  Serial.println(buf);
  TFT_printLine(buf);

  WiFi.hostname(buf);
  WiFi.begin();
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.autoConnect(HOSTNAME);

  if (WiFi.status() != WL_CONNECTED)
  {
    ESP.restart();
  }
}

// // timer fires about once per second.  Keep a counter going and every 600 seconds or so stick an action in the queue.
// void ICACHE_RAM_ATTR isrFunc()
// {

// }

void tftLoadFonts()
{
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS initialisation failed!");
    while (1)
      yield(); // Stay here twiddling thumbs waiting
  }
  Serial.println("\r\nSPIFFS available!");

  // ESP32 will crash if any of the fonts are missing
  bool font_missing = false;
  if (SPIFFS.exists("/NotoSansBold15.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/NotoSansBold36.vlw") == false)
    font_missing = true;

  if (font_missing)
  {
    Serial.println("\r\nFont missing in SPIFFS, did you upload it?");
    while (1)
      yield();
  }
  else
    Serial.println("\r\nFonts found OK.");
}

void timerCallback(void *arg)
{
  static long timerCalls;

  // Every x cycled this happens
  if (timerCalls % 50 == 0)
  {
    timerElapsed++;
    screenUpdate = true;
  }

  adcTimer = true;

  timerCalls++;
}

void setupTimer()
{
  // Initialize the timer
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = &timerCallback;
  timerArgs.arg = nullptr;
  esp_timer_handle_t timerHandle;
  esp_timer_create(&timerArgs, &timerHandle);

  // Start the timer with a period of 100 th second (in microseconds)
  esp_timer_start_periodic(timerHandle, 10000);
}

int SineValues[256]; // an array to store our values for sine

void setup()
{
  // setup telnet debug
  debug.setStoreOffline(true);
  debug.begin(115200);
  debug.println();

  // initialise tft
  if (TFT_BL > 0)
  {                                         // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    pinMode(TFT_BL, OUTPUT);                // Set backlight pin to output mode
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  setupTimer();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK); // Clear the screen to black
  debug.println("info: Screen Init");

  // Initialize the ADC
  analogReadResolution(ADC_RES);
  analogSetAttenuation(ADC_11db); // Set the input attenuation to 11 dB (for input voltages up to 3.6V)
  espNow.init();

}

void array_to_string(byte array[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
    buffer[i * 2 + 1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
  }
  // buffer[len*2] = '\0';
}

void drawHorizontalGauge(int x, int y, int width, int height, float currentValue, float maxValue)
{
  static float lastFuelValue = -1;

  if (abs(currentValue - lastFuelValue) >= 0.1 || lastFuelValue < 0)
  {
    tft.fillRect(x, y, width, height, TFT_BLACK);
    tft.fillRect(x + 2, y + 2, map(currentValue, 0, maxValue, 0, width - 4), height - 4, TFT_ORANGE);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("0", x, y + height + 10);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(String(maxValue), x + width, y + height + 10);

    // Draw markers
    tft.setTextSize(1);
    tft.setTextDatum(BL_DATUM);
    for (int i = 30; i <= maxValue; i += 20)
    {
      int xPos = map(i, 0, maxValue, 0, width - 4) + x + 2;
      tft.drawLine(xPos, y + height - 2, xPos, y + height + 2, TFT_WHITE);
      tft.drawString(String(i), xPos, y + height + 10);
    }

    if (currentValue < 10)
    {
      tft.fillRect(x + 2, y + 2, map(currentValue, 0, maxValue, 0, width - 4), height - 4, TFT_RED);
    }
    else if (currentValue < 20)
    {
      tft.fillRect(x + 2, y + 2, map(currentValue, 0, maxValue, 0, width - 4), height - 4, TFT_ORANGE);
    }

    lastFuelValue = currentValue;
  }
}

void updateScreen(int frame)
{
  SensorData data;
  
  static bool fuelQtyError, fuelPressError, oilPressError, oilTempError;
  static float batteryVoltage, fuelPress, fuelLitres, oilTemp, oilPress;

  batteryVoltage = -3E-07 * pow(filterBat.Current(), 2) + 0.00592 * filterBat.Current() - 1.0495;
  //= -2E-05x2 - 0.0178x + 108.95
  // scale the ADC so if voltage lower than 14 the readings will scale
  // 14 is the nominal calibrated system voltage
  float scaledFuel = filterCh1.Current() * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  // work out the fuel
  fuelLitres = -2E-05 * pow(scaledFuel, 2) - 0.0178 * scaledFuel + 105.95;

  //-1E-05x2 - 0.0233x + 169.87
  float scaledTemp = filterCh3.Current() * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  oilTemp = (-0.000011 * pow(scaledTemp, 2)) - (0.0233 * scaledTemp) + 167.87;

  // fuel press: = -7.50346E-05x2 - 0.066831183x + 348.8144776
  // min 34psi
  float scaledFPress = filterCh2.Current() * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  fuelPress = (-7.50346E-05 * pow(scaledFPress, 2)) - (0.066831183 * scaledFPress) + 348.8144776;

  // oil press: =  -7.77482E-10x3 + 9.95165E-07x2 - 0.002652107x + 5.992594213
  float scaledOPress = filterCh4.Current() * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  oilPress = (-7.77482E-10 * pow(scaledOPress, 3)) + (9.95165E-07 * pow(scaledOPress, 2)) - (0.002652107 * scaledOPress) + 5.992594213;

  fuelQtyError = fuelPressError = oilPressError = oilTempError = false;

  if (fuelLitres < 0)
  {
    fuelLitres = 0;
    fuelQtyError = true;
  }
  if (fuelLitres > 120)
  {
    fuelLitres = 120;
    fuelQtyError = true;
  }

  if (oilTemp < 0)
  {
    oilTemp = 0;
    oilTempError = true;
  }
  if (oilTemp > 150)
  {
    oilTemp = 150;
    oilTempError = true;
  }

  if (fuelPress < 0)
  {
    fuelPress = 0;
    fuelPressError = true;
  }
  if (fuelPress > 360)
  {
    fuelPress = 350;
    fuelPressError = true;
  }

  if (oilPress < 0)
  {
    oilPress = 0;
    oilPressError = true;
  }
  if (oilPress > 7)
  {
    oilPress = 7;
    oilPressError = true;
  }

  //-3E-07x2 + 0.006x - 1.0495

  int xLocation = 0, yLocation = 0, yIncrement = 20;
  tft.setTextSize(2);

  sprintf(string, "Bus V:%0.1f (%i)  ", batteryVoltage + 0.05, frame);
  tft.setCursor(xLocation, yLocation);
  tft.print(string);
  // TFT_printLine(string, true);

  sprintf(string, "F Qty:%i %i   ", (int)fuelLitres, fuelQtyError);
  yLocation += yIncrement;
  tft.setCursor(xLocation, yLocation);
  tft.print(string);

  // TFT_printLine(string, false);

  sprintf(string, "F Prs:%i %i   ", (int)fuelPress, fuelPressError);
  yLocation += yIncrement;
  tft.setCursor(xLocation, yLocation);
  tft.print(string);
  // TFT_printLine(string, false);

  sprintf(string, "O Tmp:%i %i   ", (int)oilTemp, oilTempError);
  yLocation += yIncrement;
  tft.setCursor(xLocation, yLocation);
  tft.print(string);
  // TFT_printLine(string, false);

  sprintf(string, "O Prs:%0.1f %i   ", oilPress, oilPressError);
  yLocation += yIncrement;
  tft.setCursor(xLocation, yLocation);
  tft.print(string);
  // TFT_printLine(string, false);

  // drawHorizontalGauge(10, 40, 200, 40, fuelLitres, 120);

  data.fuelQtyError = fuelQtyError;
  data.fuelPressError = fuelPressError;
  data.oilPressError = oilPressError;
  data.oilTempError = oilTempError;
  data.batteryVoltage = batteryVoltage;
  data.fuelPress = fuelPress;
  data.fuelLitres = fuelLitres;
  data.oilTemp = oilTemp;
  data.oilPress = oilPress;
  data.frame = frame;

  espNow.sendData(&data, sizeof(data));

}

void loop()
{
  //  handle messages
  debug.handle();
  btn1.loop();
  btn2.loop();
  static int lastFilteredValue;
  static int count;

  // this will run much faster than the actual screen update
  if (adcTimer)
  {

    filterBat.Filter(analogRead(ADC_BAT));
    filterCh1.Filter(analogRead(ADC_CH1));
    filterCh2.Filter(analogRead(ADC_CH2));
    filterCh3.Filter(analogRead(ADC_CH3));
    filterCh4.Filter(analogRead(ADC_CH4));

    adcTimer = false;
  }

  if (screenUpdate)
  {
    updateScreen(count);
    screenUpdate = false;
    count++;
  }
}