#include "wifi_manager.h"
#include <WiFi.h>
#include <Arduino.h>
#include <DNSServer.h>

static DNSServer g_dns;
static const byte DNS_PORT = 53;

bool wifi_ap_init(const FSDState *state) {
    WiFi.mode(WIFI_AP);
    // softAP(ssid, password, channel, hidden, max_connection)
    bool ok = WiFi.softAP(state->wifi_ssid, state->wifi_pass, 1, state->wifi_hidden);
    if (ok) {
        Serial.printf("[WiFi] AP: \"%s\"%s IP: %s\n",
            state->wifi_ssid,
            state->wifi_hidden ? " (HIDDEN)" : "",
            WiFi.softAPIP().toString().c_str());
        Serial.println("[WiFi] Dashboard: http://192.168.4.1");
        g_dns.start(DNS_PORT, "*", WiFi.softAPIP());
        Serial.println("[WiFi] Captive Portal DNS started");
    } else {
        Serial.println("[WiFi] AP start FAILED — continuing without web");
    }
    return ok;
}

void wifi_process_dns() {
    g_dns.processNextRequest();
}
