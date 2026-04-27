#pragma once

#include "fsd_handler.h"
#include "can_driver.h"

/**
 * web_dashboard.h — HTTP + WebSocket dashboard
 *
 * HTTP  port 80 : serves the embedded HTML dashboard
 * WebSocket port 81 : pushes JSON state every 1 s;
 *                     receives control commands from the browser
 *
 * Call web_dashboard_init() once after wifi_ap_init() succeeds. It starts the
 * Web/WiFi task on Core 0. web_dashboard_update() is kept as a safe no-op for
 * API compatibility.
 */

/**
 * Initialise HTTP and WebSocket servers.
 *
 * @param state  Pointer to the shared FSDState (read + written by command handler)
 * @param can    Pointer to the active CanDriver (used by mode-toggle command)
 */
void web_dashboard_init(FSDState *state, CanDriver *can);

/** Compatibility no-op; Web work runs in the Core 0 task. */
void web_dashboard_update();
