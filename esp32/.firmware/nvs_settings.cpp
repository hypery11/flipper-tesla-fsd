/*
 * nvs_settings.cpp — Persist user toggle settings in ESP32 NVS (flash).
 *
 * Uses the Arduino Preferences library (wrapper around ESP-IDF NVS).
 * Namespace: "fsd" — keys are short (max 15 chars for NVS).
 *
 * Only boolean toggles that the user sets via the web dashboard are stored.
 * Operational state (op_mode, counters, HW version…) is NOT persisted.
 */

#include "nvs_settings.h"
#include <Preferences.h>
#include <Arduino.h>

static const char *NVS_NS = "fsd";

void nvs_settings_load(FSDState *state) {
    Preferences prefs;
    // Always open in readWrite mode — readOnly fails if namespace doesn't exist yet
    // and can keep failing on some ESP32 Arduino core versions.
    if (!prefs.begin(NVS_NS, false /* readWrite */)) {
        Serial.println("[NVS] FATAL: cannot open NVS namespace");
        return;
    }

    state->nag_killer               = prefs.getBool("nag",        state->nag_killer);
    state->suppress_speed_chime     = prefs.getBool("chime",      state->suppress_speed_chime);
    state->force_fsd                = prefs.getBool("force_fsd",  state->force_fsd);
    state->fsd_inject_enabled       = prefs.getBool("fsd_inject", state->fsd_inject_enabled);
    state->tlssc_restore            = prefs.getBool("tlssc",      state->tlssc_restore);
    state->auto_activate_on_wake    = prefs.getBool("auto_wake",  state->auto_activate_on_wake);
    state->emergency_vehicle_detect = prefs.getBool("emerg_veh",  state->emergency_vehicle_detect);
    state->ban_shield               = prefs.getBool("ban_shield", state->ban_shield);

    prefs.end();

    Serial.printf("[NVS] Loaded: nag=%d chime=%d force=%d tlssc=%d auto_wake=%d emerg=%d\n",
        state->nag_killer, state->suppress_speed_chime, state->force_fsd,
        state->tlssc_restore, state->auto_activate_on_wake,
        state->emergency_vehicle_detect);
}

void nvs_settings_save(const FSDState *state) {
    Preferences prefs;
    if (!prefs.begin(NVS_NS, false /* readWrite */)) {
        Serial.println("[NVS] Save failed — cannot open namespace");
        return;
    }

    prefs.putBool("nag",        state->nag_killer);
    prefs.putBool("chime",      state->suppress_speed_chime);
    prefs.putBool("force_fsd",  state->force_fsd);
    prefs.putBool("fsd_inject", state->fsd_inject_enabled);
    prefs.putBool("tlssc",      state->tlssc_restore);
    prefs.putBool("auto_wake",  state->auto_activate_on_wake);
    prefs.putBool("emerg_veh",  state->emergency_vehicle_detect);
    prefs.putBool("ban_shield", state->ban_shield);

    prefs.end();
}

void nvs_wifi_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len) {
    Preferences prefs;
    if (!prefs.begin(NVS_NS, false)) return;
    // getString returns 0 and leaves buffer unchanged when key is absent
    prefs.getString("wifi_ssid", ssid, (unsigned)ssid_len);
    prefs.getString("wifi_pass", pass, (unsigned)pass_len);
    prefs.end();
    Serial.printf("[NVS] WiFi: SSID=%s\n", ssid);
}

void nvs_wifi_save(const char *ssid, const char *pass) {
    Preferences prefs;
    if (!prefs.begin(NVS_NS, false)) {
        Serial.println("[NVS] WiFi save failed — cannot open namespace");
        return;
    }
    prefs.putString("wifi_ssid", ssid);
    prefs.putString("wifi_pass", pass);
    prefs.end();
    Serial.printf("[NVS] WiFi saved: SSID=%s\n", ssid);
}
