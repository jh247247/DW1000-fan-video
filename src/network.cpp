#include "network.hpp"
#include "secrets.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <RemoteDebug.h>

RemoteDebug Debug;  


void Network::connect()
{
    Serial.println("Connecting to WiFi...");
    Serial.println(WIFI_SSID);
    // WiFi.mode(WIFI_STA);
    // WiFi.disconnect();

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int connectStart = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
        if(millis() - connectStart > 10000) {
            Serial.println("Failed to connect to WiFi");
            ESP.restart();
        }
    }
    Serial.println();

    Serial.println("Connected to the WiFi network");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // get mac address for mdns addr
    uint8_t macAddr[6];
    esp_wifi_get_mac(WIFI_IF_STA, macAddr);
    char mdnsName[20];
    sprintf(mdnsName, "esp32-%02x%02x%02x%02x%02x%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);

    // print out local dns addr
    if (!MDNS.begin(mdnsName)) {
        Serial.println("Error setting up MDNS responder!");
        while (1) {
            delay(1000);
        }
    }
    MDNS.addService("telnet", "tcp", 23);

    Serial.print("mDNS responder started with hostname: ");
    Serial.println(mdnsName);

    Debug.begin(mdnsName, RemoteDebug::VERBOSE);
    Debug.setResetCmdEnabled(true);
    Debug.showProfiler(true);
    Debug.showDebugLevel(true);
    //Debug.showColors(true);

    ArduinoOTA
        .onStart([]()
                 {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type); })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      } });

    ArduinoOTA.begin();
    ArduinoOTA.setMdnsEnabled(true);
    ArduinoOTA.setHostname(mdnsName);
}

void Network::handle() {
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected, reconnecting...");
        connect();
    }

    ArduinoOTA.handle();
    Debug.handle();
    yield();
}
