#ifndef ESPNOW_VARS_H
#define ESPNOW_VARS_H

#include <Arduino.h>
#include <esp_now.h>

// MAC:24:D7:EB:38:F8:24

uint8_t remotePeerMacAddress[] = {0x24, 0xD7, 0xEB, 0x38, 0xF8, 0x24};

class ESPNowTransmitter
{
private:
    uint8_t peerMacAddress[6];
    esp_now_peer_info_t peerInfo;

public:
    ESPNowTransmitter(uint8_t *remoteAddress)
    {
        Serial.begin(115200);
        memcpy(this->peerMacAddress, remoteAddress, 6);
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, remoteAddress, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
    }

    bool init()
    {
        WiFi.mode(WIFI_STA);
        Serial.println("initializing ESP-NOW");
        if (esp_now_init() != ESP_OK)
        {
            Serial.println("Error initializing ESP-NOW");
            return false;
        }

        if (esp_now_add_peer(&peerInfo) != ESP_OK)
        {
            Serial.println("Failed to add peer");
            return false;
        }
        return true;
    }

    void sendData(const void *data, size_t size)
    {
        esp_err_t result = esp_now_send(peerMacAddress, (uint8_t *)data, size);
        if (result == ESP_OK)
        {
            Serial.print(".");
        }
        else
        {
            Serial.println("Data transmission failed");
        }
    }
};

// uint8_t peerMacAddress[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC}; // Replace with your peer MAC address;

struct SensorData
{
    bool fuelQtyError;
    bool fuelPressError;
    bool oilPressError;
    bool oilTempError;
    float batteryVoltage;
    float fuelPress;
    float fuelLitres;
    float oilTemp;
    float oilPress;
    float amp;
    float cht1;
    bool ampError;
    int frame;
    unsigned long timestamp; //time in milliseconds
};

#endif