#pragma once

#include <WiFiS3.h>
#include "secrets.h" // Create this file with #defines for WIFI_SSID and WIFI_PASSWORD

namespace melty {
    constexpr unsigned int localPort = 2390;      // local port to listen on

    WiFiUDP Udp;

    void printWifiStatus() {
        // print the SSID of the network you're attached to:
        Serial.print("SSID: ");
        Serial.println(WiFi.SSID());

        // print your board's IP address:
        IPAddress ip = WiFi.localIP();
        Serial.print("IP Address: ");
        Serial.println(ip);

        // print the received signal strength:
        long rssi = WiFi.RSSI();
        Serial.print("signal strength (RSSI):");
        Serial.print(rssi);
        Serial.println(" dBm");
    }

    bool wifiSetup() {
        if (WiFi.status() == WL_NO_MODULE) {
            Serial.println("Communication with WiFi module failed!");
            return false;
        }

        String fv = WiFi.firmwareVersion();
        if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
            Serial.println("Please upgrade the wifi firmware");
        }

        WiFi.config(IPAddress(192,168,4,1));

        int status = WiFi.beginAP(WIFI_SSID, WIFI_PASSWORD); // #define these in secrets.h
        if (status != WL_AP_LISTENING) {
            Serial.println("Creating access point failed");
            return false;
        }

        Serial.println("Created WiFi");
        printWifiStatus();

        Serial.println("\nStarting connection to server...");
        return Udp.begin(localPort);
    }
}
