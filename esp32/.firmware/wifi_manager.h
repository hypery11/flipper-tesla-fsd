#pragma once

/**
 * wifi_manager.h — WiFi Access Point initialisation
 *
 * Starts a soft-AP with the fixed credentials below.
 * Call once from setup(); non-fatal on failure.
 */

#include "fsd_handler.h"

/** Start the WiFi AP using credentials from the state. Returns true on success. */
bool wifi_ap_init(const FSDState *state);

/** Process captive-portal DNS requests. Call from the Web/WiFi task. */
void wifi_process_dns();
