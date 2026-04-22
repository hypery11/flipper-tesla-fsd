#include "wifi_manager.h"
#include "nvs_settings.h"
#include <WiFi.h>
#include <Arduino.h>

static char g_ssid[33] = "Tesla-FSD";
static char g_pass[64] = "12345678";

bool wifi_ap_init() {
    // Load credentials from NVS (defaults kept if keys absent)
    nvs_wifi_load(g_ssid, sizeof(g_ssid), g_pass, sizeof(g_pass));

    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(g_ssid, g_pass);
    if (ok) {
        Serial.printf("[WiFi] AP: \"%s\"  IP: %s\n",
            g_ssid, WiFi.softAPIP().toString().c_str());
        Serial.println("[WiFi] Dashboard: http://192.168.4.1");
    } else {
        Serial.println("[WiFi] AP start FAILED — continuing without web");
    }
    return ok;
}

const char *wifi_ap_ssid() { return g_ssid; }
const char *wifi_ap_pass() { return g_pass; }
