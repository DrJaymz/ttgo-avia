#include <Arduino.h>
#include <ArduinoOTA.h>

#include "global.h"

void setupOTA()
{
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPort(8266);
  ArduinoOTA.onStart([]()
                     {
                       //detachInterrupt(digitalPinToInterrupt(interruptPin)); //you don't want to interrupt the update because it could brick it!

                       String type;
                       if (ArduinoOTA.getCommand() == U_FLASH)
                       {
                         type = "sketch";
                       }
                       else
                       { // U_SPIFFS
                         type = "filesystem";
                       }

                       // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                       debug.println("Start updating " + type);
                     });
  ArduinoOTA.onEnd([]()
                   { debug.println("\nEnd"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { debug.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
                       debug.printf("Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR)
                       {
                         debug.println("Auth Failed");
                       }
                       else if (error == OTA_BEGIN_ERROR)
                       {
                         debug.println("Begin Failed");
                       }
                       else if (error == OTA_CONNECT_ERROR)
                       {
                         debug.println("Connect Failed");
                       }
                       else if (error == OTA_RECEIVE_ERROR)
                       {
                         debug.println("Receive Failed");
                       }
                       else if (error == OTA_END_ERROR)
                       {
                         debug.println("End Failed");
                       }
                     });

  ArduinoOTA.begin();
  debug.println("Ready");
  debug.print("IP address: ");
  debug.println(WiFi.localIP());

    ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPort(8266);
}
