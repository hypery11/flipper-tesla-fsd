/*
 * main.cpp — Tesla FSD Unlock for ESP32
 *
 * Port of hypery11/flipper-tesla-fsd to M5Stack ATOM Lite + ATOMIC CAN Base.
 *
 * Default state: Listen-Only (blue LED).  Press button once to go Active (green).
 *
 * Button:
 *   Single click  → toggle Listen-Only / Active
 *   Long press 3s → toggle NAG Killer on/off
 *
 * Serial 115200 baud.  Status prints every 5 s when Active.
 */

#include <Arduino.h>
#include "config.h"
#include "fsd_handler.h"
#include "can_driver.h"
#include "led.h"
#include "wifi_manager.h"
#include "web_dashboard.h"
#include "nvs_settings.h"

// ── Globals ───────────────────────────────────────────────────────────────────
static CanDriver *g_can   = nullptr;
static FSDState   g_state = {};

static void apply_detected_hw(TeslaHWVersion hw, const char *reason) {
    if (hw == TeslaHW_Unknown || g_state.hw_version == hw) return;

    bool old_nag       = g_state.nag_killer;
    bool old_chime     = g_state.suppress_speed_chime;
    bool old_force     = g_state.force_fsd;
    bool old_emerg     = g_state.emergency_vehicle_detect;
    bool old_tlssc     = g_state.tlssc_restore;
    bool old_auto_wake = g_state.auto_activate_on_wake;
    OpMode old_mode    = g_state.op_mode;

    fsd_state_init(&g_state, hw);
    g_state.nag_killer              = old_nag;
    g_state.suppress_speed_chime    = old_chime;
    g_state.force_fsd               = old_force;
    g_state.emergency_vehicle_detect = old_emerg;
    g_state.tlssc_restore           = old_tlssc;
    g_state.auto_activate_on_wake   = old_auto_wake;
    g_state.op_mode                 = old_mode;

    const char *hw_str =
        (hw == TeslaHW_HW4) ? "HW4" :
        (hw == TeslaHW_HW3) ? "HW3" : "Legacy";
    Serial.printf("[HW] Auto-detected: %s (%s)\n", hw_str, reason);
}

// ── Button state machine ──────────────────────────────────────────────────────
static uint32_t g_btn_down_ms     = 0;
static uint32_t g_last_release_ms = 0;
static bool     g_btn_down        = false;
static int      g_pending_clicks  = 0;
static bool     g_long_fired      = false;  // prevent double-fire on long press

static void dispatch_clicks(int n) {
    if (n >= 1) {
        // Toggle Listen-Only ↔ Active
        if (g_state.op_mode == OpMode_ListenOnly) {
            g_state.op_mode = OpMode_Active;
            g_can->setListenOnly(false);
            Serial.println("[BTN] → Active mode");
        } else {
            g_state.op_mode = OpMode_ListenOnly;
            g_can->setListenOnly(true);
            Serial.println("[BTN] → Listen-Only mode");
        }
    }
}

static void button_tick() {
    bool pressed = (digitalRead(PIN_BUTTON) == LOW);
    uint32_t now = millis();

    if (pressed && !g_btn_down) {
        // Leading edge — debounce
        if ((now - g_last_release_ms) < BUTTON_DEBOUNCE_MS) return;
        g_btn_down      = true;
        g_btn_down_ms   = now;
        g_long_fired    = false;
    }

    if (g_btn_down && pressed && !g_long_fired) {
        // Still held — check for long press threshold
        if ((now - g_btn_down_ms) >= LONG_PRESS_MS) {
            g_long_fired      = true;
            g_pending_clicks  = 0;  // cancel any pending click
            g_state.nag_killer = !g_state.nag_killer;
            Serial.printf("[BTN] NAG Killer: %s\n", g_state.nag_killer ? "ON" : "OFF");
            nvs_settings_save(&g_state);
        }
    }

    if (!pressed && g_btn_down) {
        // Trailing edge
        g_btn_down        = false;
        g_last_release_ms = now;
        if (!g_long_fired) {
            g_pending_clicks++;
        }
    }

    // Flush pending clicks after the double-click window closes
    if (g_pending_clicks > 0 && !g_btn_down &&
        (now - g_last_release_ms) >= DOUBLE_CLICK_MS) {
        dispatch_clicks(g_pending_clicks);
        g_pending_clicks = 0;
    }
}

// ── LED refresh ───────────────────────────────────────────────────────────────
static void update_led() {
    if (g_state.rx_count == 0 && millis() > WIRING_WARN_MS) {
        led_set(LED_RED);
    } else if (g_state.car_asleep) {
        led_set(LED_PURPLE);
    } else if (g_state.tesla_ota_in_progress) {
        led_set(LED_YELLOW);
    } else if (g_state.op_mode == OpMode_Active) {
        led_set(LED_GREEN);
    } else {
        led_set(LED_BLUE);
    }
}

// ── CAN frame dispatcher ──────────────────────────────────────────────────────
static void process_frame(const CanFrame &frame) {
    g_state.rx_count++;

    // Track first frame timestamp for fallback HW detection delay
    if (g_state.rx_count == 1)
        g_state.first_rx_ms = millis();

    if (frame.id == CAN_ID_GTW_CAR_STATE)  g_state.seen_gtw_car_state++;
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) g_state.seen_gtw_car_config++;
    if (frame.id == CAN_ID_AP_CONTROL)     g_state.seen_ap_control++;

    // DLC sanity: skip zero-length frames
    if (frame.dlc == 0) return;

    // ── HW auto-detect (passive, runs in both modes) ─────────────────────────
    // 0x398 is the AUTHORITATIVE source — always overrides fallback detections.
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) {
        TeslaHWVersion hw = fsd_detect_hw_version(&frame);
        if (hw != TeslaHW_Unknown) {
            if (!g_state.hw_from_authoritative || g_state.hw_version != hw) {
                if (!g_state.hw_from_authoritative && g_state.hw_version != TeslaHW_Unknown &&
                    g_state.hw_version != hw) {
                    const char *old_str =
                        (g_state.hw_version == TeslaHW_HW4) ? "HW4" :
                        (g_state.hw_version == TeslaHW_HW3) ? "HW3" : "Legacy";
                    Serial.printf("[HW] Correcting fallback %s → 0x398 authoritative\n", old_str);
                }
                apply_detected_hw(hw, "0x398");
                g_state.hw_from_authoritative = true;
                g_state.hw_fallback_count_399 = 0;
            }
        }
        return;
    }

    // ── OTA monitoring (always, mode-independent) ─────────────────────────────
    if (frame.id == CAN_ID_GTW_CAR_STATE) {
        bool was_ota = g_state.tesla_ota_in_progress;
        fsd_handle_gtw_car_state(&g_state, &frame);
        if (!was_ota && g_state.tesla_ota_in_progress)
            Serial.printf("[OTA] Update in progress (raw=%u) - TX suspended\n", g_state.ota_raw_state);
        else if (was_ota && !g_state.tesla_ota_in_progress)
            Serial.printf("[OTA] Update finished (raw=%u) - TX resumed\n", g_state.ota_raw_state);
        return;
    }

    // ── Beyond here only run when TX is allowed ───────────────────────────────
    bool tx = fsd_can_transmit(&g_state);

    // NAG killer — build echo and send before the real frame propagates (0x370)
    if (frame.id == CAN_ID_EPAS_STATUS) {
        CanFrame echo;
        if (fsd_handle_nag_killer(&g_state, &frame, &echo) && tx)
            g_can->send(echo);
        return;
    }

    // Legacy stalk (0x045) — updates speed_profile, no TX
    if (frame.id == CAN_ID_STW_ACTN_RQ && g_state.hw_version == TeslaHW_Legacy) {
        fsd_handle_legacy_stalk(&g_state, &frame);
        return;
    }

    // Legacy autopilot control (0x3EE)
    if (frame.id == CAN_ID_AP_LEGACY && g_state.hw_version == TeslaHW_Legacy) {
        CanFrame f = frame;
        if (fsd_handle_legacy_autopilot(&g_state, &f) && tx)
            g_can->send(f);
        return;
    }

    // Fallback HW detection when 0x398 is unavailable on the tapped bus.
    // Only runs when 0x398 hasn't provided an authoritative answer yet,
    // and after a grace period to let 0x398 arrive first.
    if (!g_state.hw_from_authoritative && g_state.hw_version == TeslaHW_Unknown) {
        uint32_t age = millis() - g_state.first_rx_ms;
        if (age >= HW_FALLBACK_DELAY_MS) {
            if (frame.id == CAN_ID_AP_LEGACY) {
                apply_detected_hw(TeslaHW_Legacy, "fallback:0x3EE");
            } else if (frame.id == CAN_ID_ISA_SPEED) {
                // Require multiple 0x399 frames to avoid false HW4 on HW3 vehicles
                g_state.hw_fallback_count_399++;
                if (g_state.hw_fallback_count_399 >= HW_FALLBACK_CONFIRM)
                    apply_detected_hw(TeslaHW_HW4, "fallback:0x399");
            } else if (frame.id == CAN_ID_AP_CONTROL) {
                // 0x3FD exists on HW3/HW4. Prefer HW3 as safe default until 0x399 appears.
                apply_detected_hw(TeslaHW_HW3, "fallback:0x3FD");
            }
        }
    }

    // ISA speed chime (0x399, HW4 only)
    if (frame.id == CAN_ID_ISA_SPEED &&
        g_state.hw_version == TeslaHW_HW4 &&
        g_state.suppress_speed_chime) {
        CanFrame f = frame;
        if (fsd_handle_isa_speed_chime(&f) && tx)
            g_can->send(f);
        return;
    }

    // Follow distance → speed_profile (0x3F8), no TX
    if (frame.id == CAN_ID_FOLLOW_DIST) {
        fsd_handle_follow_distance(&g_state, &frame);
        return;
    }

    // TLSSC Restore (0x331) — DAS config spoof
    if (frame.id == CAN_ID_DAS_AP_CONFIG) {
        CanFrame f = frame;
        if (fsd_handle_tlssc_restore(&g_state, &f) && tx)
            g_can->send(f);
        return;
    }

    // HW3/HW4 autopilot control (0x3FD) — main FSD activation frame
    if (frame.id == CAN_ID_AP_CONTROL) {
        CanFrame f = frame;
        if (fsd_handle_autopilot_frame(&g_state, &f) && tx)
            g_can->send(f);
        return;
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("\n============================");
    Serial.println(" Tesla FSD Unlock — ESP32   ");
    Serial.println("============================");
    Serial.printf("[FSD] Build: %s %s\n", __DATE__, __TIME__);
#if defined(CAN_DRIVER_TWAI)
    Serial.println("[CAN] Driver: ESP32 TWAI (M5Stack ATOM Lite + ATOMIC CAN Base)");
#elif defined(CAN_DRIVER_MCP2515)
    Serial.println("[CAN] Driver: MCP2515 via SPI");
#endif

    pinMode(PIN_BUTTON, INPUT_PULLUP);
    led_init();

    fsd_state_init(&g_state, TeslaHW_Unknown);
    // Explicit safe defaults — will be overridden after HW auto-detect
    g_state.op_mode               = OpMode_ListenOnly;
    g_state.nag_killer            = true;
    g_state.suppress_speed_chime  = true;
    g_state.auto_activate_on_wake = false;
    g_state.emergency_vehicle_detect = false;
    g_state.force_fsd             = false;
    g_state.last_rx_ms            = millis();  // prevent false sleep on boot
    g_state.first_rx_ms           = millis();

    // Load persisted settings from NVS (overrides defaults above)
    nvs_settings_load(&g_state);

    led_set(LED_BLUE);

    g_can = can_driver_create();
    if (!g_can->begin(true)) {
        Serial.println("[ERR] CAN driver init FAILED — check wiring");
        led_set(LED_RED);
        // Halt: signal error via blinking red indefinitely
        while (true) {
            led_set(LED_RED);   delay(200);
            led_set(LED_OFF);   delay(200);
        }
    }

    Serial.println("[CAN] 500 kbps — Listen-Only");
    Serial.println("[BTN] Single click : toggle Listen-Only / Active");
    Serial.println("[BTN] Long press 3s: toggle NAG Killer");
    Serial.println("[LED] Blue=Listen  Green=Active  Yellow=OTA  Purple=Sleep  Red=Error");
    // If auto-activate on wake is enabled, start directly in Active mode
    if (g_state.auto_activate_on_wake) {
        g_state.op_mode = OpMode_Active;
        g_can->setListenOnly(false);
        Serial.println("[BOOT] Auto-activate enabled \u2192 Active mode");
    }
    // ── WiFi AP + Web dashboard (non-fatal if WiFi fails) ─────────────────────
    if (wifi_ap_init()) {
        web_dashboard_init(&g_state, g_can);
    }
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    button_tick();

    // Drain all available CAN frames in one shot
    CanFrame frame;
    while (g_can->receive(frame)) {
        // ── Wake-up detection: first frame after sleep ────────────────────────
        if (g_state.car_asleep) {
            g_state.car_asleep = false;
            Serial.println("[WAKE] CAN traffic resumed — car is awake");
            // Reinitialise CAN driver to clear any bus-off / error state
            g_can->reset();
            // Reset HW detection so 0x398 can re-identify correctly
            g_state.hw_version            = TeslaHW_Unknown;
            g_state.hw_from_authoritative = false;
            g_state.hw_fallback_count_399 = 0;
            g_state.fsd_enabled           = false;
            g_state.nag_suppressed        = false;
            g_state.first_rx_ms           = now;
            // Auto-activate if enabled
            if (g_state.auto_activate_on_wake && g_state.op_mode == OpMode_ListenOnly) {
                g_state.op_mode = OpMode_Active;
                g_can->setListenOnly(false);
                Serial.println("[WAKE] Auto-activate → Active mode");
            }
        }
        g_state.last_rx_ms = now;
        process_frame(frame);
    }

    // ── Car sleep detection ───────────────────────────────────────────────────
    if (g_state.rx_count > 0 && !g_state.car_asleep &&
        (now - g_state.last_rx_ms) >= CAR_SLEEP_TIMEOUT_MS) {
        g_state.car_asleep = true;
        Serial.println("[SLEEP] No CAN traffic — car is asleep, TX suspended");
    }

    // ── Periodic error counter refresh (~every 250 ms) ────────────────────────
    static uint32_t last_err_ms = 0;
    if ((now - last_err_ms) >= 250u) {
        g_state.crc_err_count = g_can->errorCount();
        last_err_ms = now;
    }

    // ── Active-mode status line ───────────────────────────────────────────────
    static uint32_t last_status_ms = 0;
    if (g_state.op_mode == OpMode_Active &&
        (now - last_status_ms) >= STATUS_PRINT_MS) {
        const char *hw_str =
            (g_state.hw_version == TeslaHW_HW4)    ? "HW4"    :
            (g_state.hw_version == TeslaHW_HW3)    ? "HW3"    :
            (g_state.hw_version == TeslaHW_Legacy)  ? "Legacy" : "?";
        Serial.printf(
            "[STA] HW:%-6s FSD:%-4s NAG:%-10s OTA:%-3s Sleep:%-3s "
            "Profile:%d  RX:%lu TX:%lu Err:%lu\n",
            hw_str,
            g_state.fsd_enabled     ? "ON"         : "wait",
            g_state.nag_suppressed  ? "suppressed"  : "active",
            g_state.tesla_ota_in_progress ? "YES"  : "no",
            g_state.car_asleep            ? "YES"  : "no",
            g_state.speed_profile,
            (unsigned long)g_state.rx_count,
            (unsigned long)g_state.frames_modified,
            (unsigned long)g_state.crc_err_count);
        last_status_ms = now;
    }

    // ── Wiring sanity warning ─────────────────────────────────────────────────
    static uint32_t last_warn_ms = 0;
    if (g_state.rx_count == 0 && now > WIRING_WARN_MS &&
        (now - last_warn_ms) >= 2000u) {
        Serial.println("[WARN] No CAN traffic after 5 s — check wiring");
        Serial.println("[WARN] Verify CAN-H on OBD pin 6, CAN-L on pin 14");
        last_warn_ms = now;
    }

    // ── Web dashboard (after CAN to preserve CAN frame latency) ──────────────
    web_dashboard_update();

    update_led();
}
