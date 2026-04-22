#pragma once

#include "fsd_handler.h"
#include <stddef.h>

/** Load persisted toggle settings from NVS into state.
 *  Missing keys use the current state values as defaults. */
void nvs_settings_load(FSDState *state);

/** Save all toggle settings from state to NVS. */
void nvs_settings_save(const FSDState *state);

/**
 * Load WiFi AP credentials from NVS.
 * Buffers must be pre-filled with defaults — unchanged if key absent.
 * ssid: max 33 bytes (32 + NUL), pass: max 64 bytes (63 + NUL).
 */
void nvs_wifi_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len);

/** Save WiFi AP credentials to NVS. */
void nvs_wifi_save(const char *ssid, const char *pass);
