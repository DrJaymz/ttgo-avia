#include <Arduino.h>
#include <Ticker.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "esp_adc_cal.h"
#include <FS.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <esp32-hal.h>
#include "WiFiManager.h" //https://github.com/tzapu/WiFiManager

#include "Filter.h"
#include "global.h"
#include "ota.h"
#include "lcd.h"
#include "Button2.h"
#include "espNow.h"
#include "max6675.h"

#include <esp32-hal-timer.h>
#include <driver/dac.h>
#include <math.h>
#include <RunningMedian.h>

// other crap
char string[16];
TelnetSpy debug;

uint8_t kRemotePeerMacAddress[6] = {0x24, 0xD7, 0xEB, 0x38, 0xF8, 0x24};

#define DAC_CH1 25
#define ADC_RES 12
// #define ADC_FILTER 1 // new value weight out of 100
// #define BUS_FILTER 0.1
#define UPDATE_RATE_USEC 10000
#define UPDATE_RATE = (1000000 / UPDATE_RATE_USEC) // samples per second
#define SIMULATE 1

// Keep ranges aligned with ../Avia Display/src/global.h
constexpr float kFuelQtyMin = 0.0f;
constexpr float kFuelQtyMax = 120.0f;
constexpr float kFuelPressMin = 0.0f;
constexpr float kFuelPressMax = 350.0f;
constexpr float kOilTempMin = 0.0f;
constexpr float kOilTempMax = 140.0f;
constexpr float kOilPressMin = 0.0f;
constexpr float kOilPressMax = 7.0f;
constexpr float kChtMin = 0.0f;
constexpr float kChtMax = 250.0f;

// filtering
#define MEDIAN_FILTER_SAMPLES 99
#define MEDIAN_FILTER_AVG_SAMPLES 5 // the number of sample that we are going to get our value from.

RunningMedian filterBat = RunningMedian(MEDIAN_FILTER_SAMPLES);
RunningMedian filterAmp = RunningMedian(MEDIAN_FILTER_SAMPLES);
RunningMedian filterCh1 = RunningMedian(MEDIAN_FILTER_SAMPLES);
RunningMedian filterCh2 = RunningMedian(MEDIAN_FILTER_SAMPLES);
RunningMedian filterCh3 = RunningMedian(MEDIAN_FILTER_SAMPLES);
RunningMedian filterCh4 = RunningMedian(MEDIAN_FILTER_SAMPLES);
RunningMedian filterCht1 = RunningMedian(15);

int ampOffset = 0;
bool ampReadingValid = false;

ESPNowTransmitter espNow(kRemotePeerMacAddress);
MAX6675 t(THERM_SCK, THERM_CS, THERM_SO);

// MAX6675 thermocouple(THERM_SCK, THERM_CS, THERM_SO);

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

  // readings are read 100 / sec, but the screen updates on 10 / sec.  The screen update also is the transmission data frame rate
  if (timerCalls % 10 == 0)
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
  esp_timer_start_periodic(timerHandle, UPDATE_RATE_USEC);
}

// get die temperature
float readEspTemp()
{
  float celsius = ((temprature_sens_read() - 32) / 1.8) - 32;
  return celsius;
}

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

  // debug.printf("i: thermocouple: %f\n",thermocouple.readCelsius());
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

  static bool fuelQtyError, fuelPressError, oilPressError, oilTempError, ampError, cht1Error;
  static float batteryVoltage, fuelPress, fuelLitres, oilTemp, oilPress, amp, cht1;

  // amp is linear
  amp = ((0.0306 * (filterAmp.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES) - ampOffset) - 0.0454));

  batteryVoltage = -3E-07 * pow(filterBat.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES), 2) + 0.00592 * filterBat.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES) - 1.0495;
  //= -2E-05x2 - 0.0178x + 108.95
  // scale the ADC so if voltage lower than 14 the readings will scale
  // 14 is the nominal calibrated system voltage
  float scaledFuel = filterCh1.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES) * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  // work out the fuel
  fuelLitres = -2E-05 * pow(scaledFuel, 2) - 0.0178 * scaledFuel + 105.95;

  //-1E-05x2 - 0.0233x + 169.87
  float scaledTemp = filterCh3.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES) * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  oilTemp = (-0.000011 * pow(scaledTemp, 2)) - (0.0233 * scaledTemp) + 167.87;

  // fuel press: = -7.50346E-05x2 - 0.066831183x + 348.8144776
  // min 34psi
  float scaledFPress = filterCh2.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES) * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  fuelPress = (-7.50346E-05 * pow(scaledFPress, 2)) - (0.066831183 * scaledFPress) + 348.8144776;

  // oil press: =  -7.77482E-10x3 + 9.95165E-07x2 - 0.002652107x + 5.992594213
  float scaledOPress = filterCh4.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES) * ((14.0 / batteryVoltage)) - ((batteryVoltage - 14.0) * 15);
  oilPress = (-7.77482E-10 * pow(scaledOPress, 3)) + (9.95165E-07 * pow(scaledOPress, 2)) - (0.002652107 * scaledOPress) + 5.992594213;

  cht1 = filterCht1.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES);
  if (cht1 < kChtMin || cht1 > kChtMax)
  {
    if (cht1 < kChtMin)
      cht1 = kChtMin;
    else
      cht1 = kChtMax;
    cht1Error = true;
  }

  fuelQtyError = fuelPressError = oilPressError = oilTempError = cht1Error = false;

  if (SIMULATE)
  {
    const float t = millis() / 1000.0f;
    auto wave = [](float x, float minV, float maxV, float spanScale) {
      float s = 0.5f + 0.5f * sinf(x);
      float mid = (minV + maxV) * 0.5f;
      float span = (maxV - minV) * 0.5f * spanScale;
      return mid + (s * 2.0f - 1.0f) * span;
    };

    const float spanScale = 0.1f; // 10% of full range for smaller steps
    batteryVoltage = wave(t * 0.5f, 11.0f, 14.5f, spanScale);
    fuelLitres = wave(t * 0.2f + 1.0f, kFuelQtyMin, kFuelQtyMax, spanScale);
    fuelPress = wave(t * 0.35f + 2.0f, kFuelPressMin, kFuelPressMax, spanScale);
    oilTemp = wave(t * 0.25f + 3.0f, kOilTempMin, kOilTempMax, spanScale);
    oilPress = wave(t * 0.4f + 4.0f, kOilPressMin, kOilPressMax, spanScale);
    cht1 = wave(t * 0.3f + 5.0f, kChtMin, kChtMax, spanScale);
    amp = wave(t * 0.6f + 6.0f, -20.0f, 20.0f, spanScale);

    ampReadingValid = true;
  }

  if (fuelLitres < 0)
  {
    fuelLitres = kFuelQtyMin;
    fuelQtyError = true;
  }
  if (fuelLitres > kFuelQtyMax)
  {
    fuelLitres = kFuelQtyMax;
    fuelQtyError = true;
  }

  if (oilTemp < 0)
  {
    oilTemp = kOilTempMin;
    oilTempError = true;
  }
  if (oilTemp > kOilTempMax)
  {
    oilTemp = kOilTempMax;
    oilTempError = true;
  }

  if (fuelPress < 0)
  {
    fuelPress = kFuelPressMin;
    fuelPressError = true;
  }
  if (fuelPress > kFuelPressMax)
  {
    fuelPress = kFuelPressMax;
    fuelPressError = true;
  }

  if (oilPress < 0)
  {
    oilPress = kOilPressMin;
    oilPressError = true;
  }
  if (oilPress > kOilPressMax)
  {
    oilPress = kOilPressMax;
    oilPressError = true;
  }

  ampError = !ampReadingValid;

  //-3E-07x2 + 0.006x - 1.0495

  int xLocation = 0, yLocation = 0, yIncrement = 18;
  tft.setTextSize(2);

  sprintf(string, "Bus V:%0.1f (%i)  ", batteryVoltage + 0.05, frame);
  tft.setCursor(xLocation, yLocation);
  tft.print(string);

  // Flashing red S indicator when simulating.
  if (SIMULATE)
  {
    const bool blinkOn = ((millis() / 500) % 2) == 0;
    const int sW = 12;
    const int sH = 16;
    const int sx = tft.width() - sW;
    const int sy = 0;
    tft.fillRect(sx, sy, sW, sH, TFT_BLACK);
    if (blinkOn)
    {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.setTextSize(2);
      tft.setCursor(sx, sy);
      tft.print("S");
    }
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
  }
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

  sprintf(string, "cht1:%0i  %0.0f ", (int)cht1, readEspTemp());
  yLocation += yIncrement;
  tft.setCursor(xLocation, yLocation);
  tft.print(string);

  if (ampReadingValid)
  {
    sprintf(string, "Amp:%0.1f   ", amp);
    yLocation += yIncrement;
    tft.setCursor(xLocation, yLocation);
    tft.print(string);
  }

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
  data.amp = (int)amp + 0.5;
  data.ampError = ampError;
  data.cht1 = cht1;

  espNow.sendData(&data, sizeof(data));
}

float getRandomFloat()
{
  long randNumber = random(0, 601);            // Generates a number from 0 to 600
  float scaled = (randNumber / 1000.0f) * 0.6; // Scale it down to 0.0 - 0.6
  return scaled + 0.7;                         // Shift the scale to 0.7 - 1.3
}

void loop()
{
  //  handle messages
  debug.handle();
  btn1.loop();
  btn2.loop();
  static int lastFilteredValue;
  static int count;
  static bool firstRun = false;
  static long nextTempReading = millis() + 1000;
  static long nextSimulatedReading = millis() + 10000;
  static float simMultiplier = 1.0;

  if (SIMULATE)
  {
    if (millis() > nextSimulatedReading)
    {
      simMultiplier = getRandomFloat();
      nextSimulatedReading = millis() + 10000;
    }
  }

  if (!ampReadingValid && millis() > 10000)
  {
    ampOffset = filterAmp.getMedianAverage(MEDIAN_FILTER_AVG_SAMPLES);
    ampReadingValid = true;
  }

  // this will run much faster than the actual screen update
  if (adcTimer)
  {

    if (SIMULATE)
    {

      filterBat.add(2744);
      filterCh1.add(1388 * simMultiplier); // fuel
      filterCh2.add(755 * simMultiplier);  // f press
      filterCh3.add(1904 * simMultiplier); // oil T
      filterCh4.add(407 * simMultiplier);  // oil press
      filterAmp.add(1500 * simMultiplier);
      filterCht1.add(150 * simMultiplier);
      firstRun = false;
    }
    else
    {
      if (firstRun)
      {
        filterBat.add(analogRead(ADC_BAT));
        filterCh1.add(analogRead(ADC_CH1));
        filterCh2.add(analogRead(ADC_CH2));
        filterCh3.add(analogRead(ADC_CH3));
        filterCh4.add(analogRead(ADC_CH4));
        filterAmp.add(analogRead(ADC_AMP));
        filterCht1.add(t.readCelsius() + CHT1_COMPENSATION);
        firstRun = false;
      }
      else
      {
        filterBat.add(analogRead(ADC_BAT));
        filterCh1.add(analogRead(ADC_CH1));
        filterCh2.add(analogRead(ADC_CH2));
        filterCh3.add(analogRead(ADC_CH3));
        filterCh4.add(analogRead(ADC_CH4));
        filterAmp.add(analogRead(ADC_AMP));

        // don't read the temp reading too often
        if (millis() > nextTempReading)
        {
          float cht1 = t.readCelsius() + CHT1_COMPENSATION;
          if (!isnanf(cht1))
          {
            filterCht1.add(cht1);
          }
          else
          {
            filterCht1.clear();
          }

          nextTempReading = millis() + 250;
        }
      }
    }

    adcTimer = false;
  }

  if (screenUpdate)
  {
    updateScreen(count);
    screenUpdate = false;
    count++;
  }
}
