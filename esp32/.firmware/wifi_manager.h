#pragma once

/**
 * wifi_manager.h — WiFi Access Point initialisation
 *
 * Credentials are loaded from NVS on init; defaults: "Tesla-FSD" / "12345678".
 * Call wifi_ap_init() once from setup(); non-fatal on failure.
 */

/** Start the WiFi AP. Returns true on success. */
bool wifi_ap_init();

/** Return the SSID currently in use by the AP. */
const char *wifi_ap_ssid();

/** Return the password currently in use by the AP. */
const char *wifi_ap_pass();
