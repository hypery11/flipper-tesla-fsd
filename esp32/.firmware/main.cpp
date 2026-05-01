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
 *   Double click  → toggle BMS serial output
 *
 * Serial 115200 baud.  Status prints every 5 s when Active.
 * BMS output (when enabled): voltage, current, power, SoC, temp every 1 s.
 */

#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include "config.h"
#include "fsd_handler.h"
#include "can_driver.h"
#include "led.h"
#include "wifi_manager.h"
#include "web_dashboard.h"
#include "can_dump.h"
#include "prefs.h"

// ── Globals ───────────────────────────────────────────────────────────────────
static CanDriver *g_can   = nullptr;
static FSDState   g_state = {};

static void apply_detected_hw(TeslaHWVersion hw, const char *reason) {
    if (hw == TeslaHW_Unknown || g_state.hw_version == hw) return;
    // If the user has pinned a HW version via the dashboard, ignore auto-
    // detect updates. Useful when the OBD-II Party CAN tap doesn't carry
    // 0x398 and the 0x399-fallback misclassifies the car.
    if (g_state.hw_override != TeslaHW_Unknown) {
        Serial.printf("[HW] Auto-detect %s ignored (override pinned)\n", reason);
        return;
    }

    fsd_apply_hw_version(&g_state, hw);

    const char *hw_str =
        (hw == TeslaHW_HW4) ? "HW4" :
        (hw == TeslaHW_HW3) ? "HW3" : "Legacy";
    Serial.printf("[HW] Auto-detected: %s (%s)\n", hw_str, reason);
    can_dump_log("HW  auto-detected: %s (%s)", hw_str, reason);
}

// ── Button state machine ──────────────────────────────────────────────────────
static uint32_t g_btn_down_ms     = 0;
static uint32_t g_last_release_ms = 0;
static bool     g_btn_down        = false;
static int      g_pending_clicks  = 0;
static bool     g_long_fired      = false;  // prevent double-fire on long press
static bool     g_btn_ignore_boot = true;   // wait for release after boot
static bool     g_factory_reset_window   = false;  // set true on clean boot, clears at 20s
static bool     g_factory_reset_eligible = false;  // latched at leading edge if press was in window
static bool     g_factory_reset_armed    = false;  // blink done, waiting for release

#if defined(SLEEP_STRATEGY_EXT0) || defined(SLEEP_STRATEGY_TIMER)
static uint32_t g_last_can_rx_ms = 0;
static bool     g_sleep_warned   = false;
#endif
#if defined(SLEEP_STRATEGY_TIMER)
// True for the brief listen window after a timer wake. Cleared on the first
// CAN frame seen, so the device stays awake whenever the car is alive.
static bool     g_probe_mode     = false;
// Track whether WiFi/dashboard were started this boot, so the lazy starter
// (called from setup or after a successful probe) is idempotent. RAM is wiped
// on deep sleep, so this is naturally false on every wake.
static bool     g_wifi_started   = false;

static void start_wifi_lazy() {
    if (g_wifi_started) return;
    g_wifi_started = true;
    if (wifi_ap_init(&g_state)) {
        web_dashboard_init(&g_state, g_can);
    }
}
#endif

static void dispatch_clicks(int n) {
    if (n == 1) {
        // Toggle Listen-Only ↔ Active
        if (g_state.op_mode == OpMode_ListenOnly) {
            g_state.op_mode = OpMode_Active;
            g_can->setListenOnly(false);
            Serial.println("[BTN] → Active mode");
            can_dump_log("MODE switched to Active — TX enabled");
        } else {
            g_state.op_mode = OpMode_ListenOnly;
            g_can->setListenOnly(true);
            Serial.println("[BTN] → Listen-Only mode");
            can_dump_log("MODE switched to Listen-Only — TX disabled");
        }
        prefs_save(&g_state);
    } else if (n >= 2) {
        // Toggle BMS serial output
        g_state.bms_output = !g_state.bms_output;
        Serial.printf("[BTN] BMS output: %s\n", g_state.bms_output ? "ON" : "OFF");
        prefs_save(&g_state);
    }
}

static void button_tick() {
    bool pressed = (digitalRead(PIN_BUTTON) == LOW);
    uint32_t now = millis();

    if (g_btn_ignore_boot) {
        if (!pressed) g_btn_ignore_boot = false;
        return;
    }

    if (pressed && !g_btn_down) {
        // Leading edge — debounce
        if ((now - g_last_release_ms) < BUTTON_DEBOUNCE_MS) return;
        g_btn_down             = true;
        g_btn_down_ms          = now;
        g_long_fired           = false;
        g_factory_reset_eligible = g_factory_reset_window;  // latch at press time
#if defined(SLEEP_STRATEGY_TIMER)
        // A button press during a post-wake probe is deliberate user intent;
        // clear probe mode so the normal sleep_idle_ms threshold takes over
        // and the device doesn't immediately re-sleep on probe expiry.
        if (g_probe_mode) {
            g_probe_mode = false;
            g_last_can_rx_ms = now;  // restart the sleep clock from this press
            Serial.println("[WAKE] Button press during probe — staying awake");
            start_wifi_lazy();
        }
#endif
    }

    if (g_btn_down && pressed && !g_long_fired) {
        uint32_t held = now - g_btn_down_ms;
        if (g_factory_reset_eligible && held >= FACTORY_RESET_HOLD_MS) {
            g_long_fired           = true;
            g_pending_clicks       = 0;
            g_factory_reset_armed  = true;
            Serial.println("[BTN] Factory reset armed — release to confirm");
            led_factory_blink();
        } else if (!g_factory_reset_eligible && held >= LONG_PRESS_MS) {
            g_long_fired      = true;
            g_pending_clicks  = 0;
            g_state.nag_killer = !g_state.nag_killer;
            Serial.printf("[BTN] NAG Killer: %s\n", g_state.nag_killer ? "ON" : "OFF");
            prefs_save(&g_state);
        }
        // eligible press with held < FACTORY_RESET_HOLD_MS: suppress 3s NAG killer
    }

    if (!pressed && g_btn_down) {
        // Trailing edge
        g_btn_down               = false;
        g_last_release_ms        = now;
        g_factory_reset_eligible = false;
        if (g_factory_reset_armed) {
            Serial.println("[BTN] Factory reset confirmed — clearing NVS");
            prefs_clear();
            delay(200);
            ESP.restart();
        }
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
    if (g_factory_reset_armed) {
        led_set(LED_WHITE);
        return;
    }
#if defined(SLEEP_STRATEGY_TIMER)
    if (g_probe_mode) {
        led_set(LED_SLEEP);  // dim white while listening for CAN after timer wake
        return;
    }
#endif
    if (g_state.rx_count == 0 && millis() > WIRING_WARN_MS) {
        led_set(LED_RED);
    } else if (g_state.tesla_ota_in_progress && !g_state.ota_ignore) {
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
    can_dump_record(frame);
#if defined(SLEEP_STRATEGY_EXT0) || defined(SLEEP_STRATEGY_TIMER)
    g_last_can_rx_ms = millis();
    g_sleep_warned   = false;
  #if defined(SLEEP_STRATEGY_TIMER)
    if (g_probe_mode) {
        g_probe_mode = false;
        Serial.println("[WAKE] CAN traffic detected — staying awake");
        start_wifi_lazy();
    }
  #endif
#endif

    // ── CAN serial trace: print to serial when an ID's bytes change ─────────
    // Bounded table tracks the last-seen bytes per unique ID. When can_trace
    // is on, each change emits one [TRACE] line — perfect for finding which
    // ID carries an unknown signal (e.g. brake pedal): trigger the input on
    // the car and watch which IDs print.
    {
        int slot = -1;
        for (uint16_t i = 0; i < g_state.trace_count; i++) {
            if (g_state.trace_ids[i] == (uint16_t)frame.id) { slot = (int)i; break; }
        }
        if (slot < 0 && g_state.trace_count < TRACE_MAX) {
            slot = g_state.trace_count++;
            g_state.trace_ids[slot] = (uint16_t)frame.id;
            // Fresh slot starts with sentinel so the first frame of a new ID
            // does not register as a "change" and spam serial when trace
            // is toggled on.
            g_state.trace_dlc[slot] = 0xFF;
        }
        if (slot >= 0) {
            uint8_t dlc = frame.dlc < 8 ? frame.dlc : 8;
            bool changed = (g_state.trace_dlc[slot] != 0xFF) &&
                           (g_state.trace_dlc[slot] != dlc);
            for (uint8_t i = 0; i < dlc; i++) {
                if (g_state.trace_dlc[slot] != 0xFF &&
                    g_state.trace_bytes[slot][i] != frame.data[i]) changed = true;
                g_state.trace_bytes[slot][i] = frame.data[i];
            }
            g_state.trace_dlc[slot] = dlc;
            if (changed && g_state.can_trace) {
                // Backpressure guard: if the UART TX buffer doesn't have room
                // for a full line (~40 chars), drop this print rather than
                // block the loop. Without USB connected the FIFO fills fast
                // and Serial.printf would otherwise stall web_dashboard_update.
                if (Serial.availableForWrite() >= 40) {
                    Serial.printf("[TRACE] 0x%03X dlc=%u:", (unsigned)frame.id, dlc);
                    for (uint8_t i = 0; i < dlc; i++) Serial.printf(" %02X", frame.data[i]);
                    Serial.println();
                }
            }
        }
    }

    if (frame.id == CAN_ID_GTW_CAR_STATE)  g_state.seen_gtw_car_state++;
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) g_state.seen_gtw_car_config++;
    if (frame.id == CAN_ID_AP_CONTROL)     g_state.seen_ap_control++;
    if (frame.id == CAN_ID_FOLLOW_DIST)    g_state.seen_follow_dist++;
    if (frame.id == CAN_ID_BMS_HV_BUS)     g_state.seen_bms_hv++;
    if (frame.id == CAN_ID_BMS_SOC)        g_state.seen_bms_soc++;
    if (frame.id == CAN_ID_BMS_THERMAL)    g_state.seen_bms_thermal++;

    // DLC sanity: skip zero-length frames
    if (frame.dlc == 0) return;

    // ── HW auto-detect (passive, runs in both modes) ─────────────────────────
    if (frame.id == CAN_ID_GTW_CAR_CONFIG) {
        TeslaHWVersion hw = fsd_detect_hw_version(&frame);
        if (hw != TeslaHW_Unknown && g_state.hw_version == TeslaHW_Unknown)
            apply_detected_hw(hw, "0x398");
        return;
    }

    // ── OTA monitoring (always, mode-independent) ─────────────────────────────
    if (frame.id == CAN_ID_GTW_CAR_STATE) {
        bool was_ota = g_state.tesla_ota_in_progress;
        fsd_handle_gtw_car_state(&g_state, &frame);
        if (!was_ota && g_state.tesla_ota_in_progress) {
            Serial.printf("[OTA] Update in progress (raw=%u) - TX suspended\n", g_state.ota_raw_state);
            can_dump_log("OTA  started — TX suspended");
        } else if (was_ota && !g_state.tesla_ota_in_progress) {
            Serial.printf("[OTA] Update finished (raw=%u) - TX resumed\n", g_state.ota_raw_state);
            can_dump_log("OTA  finished — TX resumed");
        }
        return;
    }

    // ── BMS sniff (read-only, always) ─────────────────────────────────────────
    if (frame.id == CAN_ID_BMS_HV_BUS)  { fsd_handle_bms_hv(&g_state, &frame);      return; }
    if (frame.id == CAN_ID_BMS_SOC)     { fsd_handle_bms_soc(&g_state, &frame);     return; }
    if (frame.id == CAN_ID_BMS_THERMAL) { fsd_handle_bms_thermal(&g_state, &frame); return; }

    // ── DAS status (read-only, always) — gating for NAG killer + diagnostics ─
    if (frame.id == CAN_ID_DAS_STATUS)  { fsd_handle_das_status(&g_state, &frame);  return; }

    // ── Vehicle dynamics (read-only, always) ─────────────────────────────────
    if (frame.id == CAN_ID_DI_SPEED)    { fsd_handle_di_speed(&g_state, &frame);    return; }
    if (frame.id == CAN_ID_ESP_STATUS)  { fsd_handle_esp_status(&g_state, &frame);  return; }
    if (frame.id == CAN_ID_DI_TORQUE)   { fsd_handle_di_torque(&g_state, &frame);   return; }
    if (frame.id == CAN_ID_STEER_ANGLE) { fsd_handle_steering_angle(&g_state, &frame); return; }
    if (frame.id == CAN_ID_BATT_STATUS) { fsd_handle_batt_status_chassis(&g_state, &frame); return; }
    if (frame.id == CAN_ID_BATT_TEMP)   { fsd_handle_batt_temp(&g_state, &frame);           return; }
    if (frame.id == CAN_ID_DC_BUS)      { fsd_handle_dc_bus(&g_state, &frame);              return; }

    // ── Beyond here only run when TX is allowed ───────────────────────────────
    bool tx = fsd_can_transmit(&g_state);

    // NAG killer — build echo and send before the real frame propagates (0x370)
    if (frame.id == CAN_ID_EPAS_STATUS) {
        CanFrame echo;
        bool fired = fsd_handle_nag_killer(&g_state, &frame, &echo);
        if (fired) {
            uint8_t lvl     = (frame.data[4] >> 6) & 0x03;
            uint8_t cnt_in  = frame.data[6] & 0x0F;
            uint8_t cnt_out = echo.data[6] & 0x0F;
            can_dump_log("NAG 0x370 hands_off lvl=%u cnt=%u->%u %s",
                         lvl, cnt_in, cnt_out, tx ? "TX echo" : "listen-only no-TX");
            if (tx) g_can->send(echo);
        }
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

    // Auto-upgrade Legacy→HW3: Palladium S/X with HW3 reports das_hw=0
    // (→Legacy) but actually uses 0x3FD. True Legacy never broadcasts 0x3FD.
    if (g_state.hw_version == TeslaHW_Legacy && frame.id == CAN_ID_AP_CONTROL) {
        apply_detected_hw(TeslaHW_HW3, "upgrade:Legacy→HW3(0x3FD seen)");
    }

    // Fallback HW detection when 0x398 is unavailable on the tapped bus.
    // Delay 0x3FD→HW3 fallback to avoid misclassifying HW4 (which also has
    // 0x3FD) before 0x399 arrives. 0x3EE and 0x399 are unambiguous.
    static uint32_t hw_fallback_3fd_count = 0;
    if (g_state.hw_version == TeslaHW_Unknown) {
        if (frame.id == CAN_ID_AP_LEGACY) {
            apply_detected_hw(TeslaHW_Legacy, "fallback:0x3EE");
        } else if (frame.id == CAN_ID_ISA_SPEED) {
            apply_detected_hw(TeslaHW_HW4, "fallback:0x399");
            hw_fallback_3fd_count = 0;
        } else if (frame.id == CAN_ID_AP_CONTROL) {
            hw_fallback_3fd_count++;
            if (hw_fallback_3fd_count >= 50)
                apply_detected_hw(TeslaHW_HW3, "fallback:0x3FD(confirmed)");
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

    // TLSSC Restore (0x331) — DAS config spoof + DAS_autopilot tier readback
    if (frame.id == CAN_ID_DAS_AP_CONFIG) {
        // Always parse the readback first so the dashboard reflects the
        // car's reported tier even when TLSSC restore is disabled.
        fsd_handle_das_ap_config(&g_state, &frame);
        g_state.seen_das_ap_config++;
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

#if defined(SLEEP_STRATEGY_EXT0)
// ── Deep-sleep watchdog (EXT0 wake on CAN_RX edge) ───────────────────────────
static void sleep_tick(uint32_t now) {
    if (now < g_last_can_rx_ms) return;
    uint32_t idle_ms = now - g_last_can_rx_ms;

    if (idle_ms >= g_state.sleep_idle_ms) {
        Serial.printf("[SLEEP] Entering deep sleep (EXT0 wake) after %lu ms CAN silence\n",
                      (unsigned long)idle_ms);
        can_dump_stop();
        sd_syslog_close();
        led_set(LED_SLEEP);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_CAN_RX, 0);
        esp_deep_sleep_start();
        // never returns
    } else if (!g_sleep_warned && idle_ms >= (g_state.sleep_idle_ms - SLEEP_WARN_MS)) {
        g_sleep_warned = true;
        uint32_t remaining_ms = g_state.sleep_idle_ms - idle_ms;
        Serial.printf("[SLEEP] Warning: %lu ms idle, sleeping in %lu ms\n",
                      (unsigned long)idle_ms, (unsigned long)remaining_ms);
    }
}
#elif defined(SLEEP_STRATEGY_TIMER)
// ── Deep-sleep watchdog (timer wake + post-wake CAN probe) ───────────────────
// In probe mode the threshold is the short SLEEP_PROBE_MS window; once a CAN
// frame arrives the probe flag clears and we revert to the user-configured
// sleep_idle_ms (driven from the web dashboard slider).
static void sleep_tick(uint32_t now) {
    if (now < g_last_can_rx_ms) return;
    uint32_t idle_ms      = now - g_last_can_rx_ms;
    uint32_t threshold_ms = g_probe_mode ? SLEEP_PROBE_MS : g_state.sleep_idle_ms;

    if (idle_ms >= threshold_ms) {
        Serial.printf("[SLEEP] Entering deep sleep (timer %us / button wake) after %lu ms idle (%s)\n",
                      (unsigned)SLEEP_TIMER_WAKE_S, (unsigned long)idle_ms,
                      g_probe_mode ? "probe miss" : "user threshold");
        can_dump_stop();
        led_set(LED_SLEEP);
        // Wake on button press (active-LOW, EXT0). Timer + EXT0 are
        // independent wake sources — either fires.
        // pullup_en/pulldown_dis are no-ops on input-only pins (GPIO 34-39)
        // which have no internal pulls; those rely on an external pull-up
        // (e.g. M5Stack ATOM PCB has one on GPIO 39).
        rtc_gpio_pullup_en((gpio_num_t)PIN_BUTTON);
        rtc_gpio_pulldown_dis((gpio_num_t)PIN_BUTTON);
        esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_BUTTON, 0);
        esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_TIMER_WAKE_S * 1000000ULL);
        esp_deep_sleep_start();
        // never returns
    } else if (!g_probe_mode && !g_sleep_warned &&
               idle_ms >= (g_state.sleep_idle_ms - SLEEP_WARN_MS)) {
        g_sleep_warned = true;
        uint32_t remaining_ms = g_state.sleep_idle_ms - idle_ms;
        Serial.printf("[SLEEP] Warning: %lu ms idle, sleeping in %lu ms\n",
                      (unsigned long)idle_ms, (unsigned long)remaining_ms);
    }
}
#endif

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
#if defined(SLEEP_STRATEGY_EXT0) || defined(SLEEP_STRATEGY_TIMER)
    g_last_can_rx_ms = millis();
#endif
#if defined(SLEEP_STRATEGY_TIMER)
    // If we just woke via EXT0 the button pin is still held by the RTC IO
    // subsystem; release it before pinMode() below so digital IO regains
    // control. Safe no-op when the pin was never in RTC mode (cold boot).
    rtc_gpio_deinit((gpio_num_t)PIN_BUTTON);
#endif
    Serial.begin(115200);
    delay(300);

    Serial.println("\n============================");
    Serial.println(" Tesla FSD Unlock — ESP32   ");
    Serial.println("============================");
    Serial.printf("[FSD] Build: %s %s\n", __DATE__, __TIME__);
#if defined(CAN_DRIVER_TWAI)
  #if defined(BOARD_WAVESHARE_S3)
    Serial.println("[CAN] Driver: ESP32-S3 TWAI (Waveshare ESP32-S3-RS485-CAN)");
  #elif defined(BOARD_LILYGO)
    Serial.println("[CAN] Driver: ESP32 TWAI (LilyGO T-CAN485)");
  #elif defined(BOARD_M5STACK_ATOM_MATRIX)
    Serial.println("[CAN] Driver: ESP32 TWAI (M5Stack ATOM Matrix + ATOMIC CAN Base)");
  #else
    Serial.println("[CAN] Driver: ESP32 TWAI (M5Stack ATOM Lite + ATOMIC CAN Base)");
  #endif
#elif defined(CAN_DRIVER_MCP2515)
    Serial.println("[CAN] Driver: MCP2515 via SPI");
#endif

#if defined(BOARD_LILYGO)
    pinMode(ME2107_EN, OUTPUT);
    digitalWrite(ME2107_EN, HIGH);
    delay(100); // Wait for 5V rail to stabilize (SD power)
    // CAN transceiver slope/mode pin — must be LOW for normal TX+RX operation.
    // Floating or HIGH puts the SN65HVD230/TJA1051 into standby (RX-only),
    // which causes the TWAI controller to go bus-off the first time it tries to TX.
    pinMode(PIN_CAN_SPEED_MODE, OUTPUT);
    digitalWrite(PIN_CAN_SPEED_MODE, LOW);
#endif

    Serial.printf("[CFG] pins: LED=%d BUTTON=%d CAN_TX=%d CAN_RX=%d\n",
                  PIN_LED, PIN_BUTTON, PIN_CAN_TX, PIN_CAN_RX);

    // ESP32 input-only pins (GPIO 34-39) have no internal pulls; the M5Stack
    // ATOM Matrix wires the front button to GPIO 39 with an external pull-up
    // on the PCB, so plain INPUT is correct. All other supported boards wire
    // the button to a regular GPIO with internal pull-up available.
#if (PIN_BUTTON >= 34 && PIN_BUTTON <= 39)
    pinMode(PIN_BUTTON, INPUT);
#else
    pinMode(PIN_BUTTON, INPUT_PULLUP);
#endif
    led_init();

    fsd_state_init(&g_state, TeslaHW_Unknown);
    // Explicit safe defaults — will be overridden after HW auto-detect
    g_state.op_mode               = OpMode_ListenOnly;
    g_state.nag_killer            = true;
    g_state.suppress_speed_chime  = true;
    g_state.emergency_vehicle_detect = false;
    g_state.force_fsd             = false;
    g_state.bms_output            = false;

    prefs_load(&g_state);

    // If the user has pinned a HW version, apply it now so subsequent
    // auto-detect attempts (which run from process_frame) are short-circuited
    // by apply_detected_hw().
    if (g_state.hw_override != TeslaHW_Unknown) {
        fsd_apply_hw_version(&g_state, g_state.hw_override);
        const char *hw_str =
            (g_state.hw_override == TeslaHW_HW4)    ? "HW4"    :
            (g_state.hw_override == TeslaHW_HW3)    ? "HW3"    :
            (g_state.hw_override == TeslaHW_Legacy) ? "Legacy" : "?";
        Serial.printf("[HW] Pinned by override: %s (auto-detect disabled)\n", hw_str);
    }

    {
        esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();
        g_factory_reset_window = (wakeup == ESP_SLEEP_WAKEUP_UNDEFINED);
        if (g_factory_reset_window)
            Serial.println("[BTN] Factory reset window active — hold button 5s within 20s");
    }

    if (g_state.op_mode == OpMode_Active) {
        // Will be re-applied after g_can is created; record intent here only
        Serial.println("[NVS] Restored Active mode from NVS");
    }

    led_set(LED_BLUE);

    can_dump_init();

#if defined(SLEEP_STRATEGY_EXT0)
    {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_EXT0) {
            Serial.printf("[WAKE] Woken by CAN activity (EXT0 GPIO %d)\n", PIN_CAN_RX);
        } else if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
            Serial.printf("[WAKE] Wakeup cause=%d\n", (int)cause);
        }
        g_last_can_rx_ms = millis();
    }
#elif defined(SLEEP_STRATEGY_TIMER)
    {
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_TIMER) {
            g_probe_mode = true;
            Serial.printf("[WAKE] Timer probe — listening %u ms for CAN traffic\n",
                          (unsigned)SLEEP_PROBE_MS);
        } else if (cause == ESP_SLEEP_WAKEUP_EXT0) {
            // Deliberate user wake via the button — skip probe and run
            // normally. RTC IO has already been released for this pin at the
            // top of setup() so pinMode could regain control.
            Serial.printf("[WAKE] Button press (GPIO %d) — staying awake\n", PIN_BUTTON);
        } else if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
            Serial.printf("[WAKE] Wakeup cause=%d\n", (int)cause);
        }
        g_last_can_rx_ms = millis();
    }
#endif

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

    if (g_state.op_mode == OpMode_Active) {
        g_can->setListenOnly(false);
        Serial.println("[CAN] 500 kbps — Active (restored from NVS)");
    } else {
        Serial.println("[CAN] 500 kbps — Listen-Only");
    }
    Serial.println("[BTN] Single click : toggle Listen-Only / Active");
    Serial.println("[BTN] Long press 3s: toggle NAG Killer");
    Serial.println("[BTN] Double click : toggle BMS serial output");
    Serial.println("[LED] Blue=Listen  Green=Active  Yellow=OTA  Red=Error");

    // ── WiFi AP + Web dashboard (non-fatal if WiFi fails) ─────────────────────
    // For TIMER-sleep boards we defer WiFi until the post-wake CAN probe
    // succeeds — saves ~50 mA × probe duration on every miss while parked.
#if defined(SLEEP_STRATEGY_TIMER)
    if (!g_probe_mode) {
        start_wifi_lazy();
    } else {
        Serial.println("[WiFi] Deferred — will start once CAN traffic confirms wake");
    }
#else
    if (wifi_ap_init(&g_state)) {
        web_dashboard_init(&g_state, g_can);
    }
#endif
}

// ── loop ──────────────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    if (g_factory_reset_window && now >= FACTORY_RESET_WINDOW_MS) {
        g_factory_reset_window = false;
        Serial.println("[BTN] Factory reset window closed");
    }

    button_tick();

    // Drain all available CAN frames in one shot
    CanFrame frame;
    while (g_can->receive(frame)) {
        process_frame(frame);
    }

    // ── Periodic error counter refresh (~every 250 ms) ────────────────────────
    static uint32_t last_err_ms = 0;
    if ((now - last_err_ms) >= 250u) {
        g_state.crc_err_count = g_can->errorCount();
        last_err_ms = now;
    }

    // ── Precondition frame injection ──────────────────────────────────────────
    static uint32_t last_precond_ms = 0;
    if (g_state.precondition && fsd_can_transmit(&g_state) &&
        (now - last_precond_ms) >= PRECOND_INTERVAL_MS) {
        CanFrame pf;
        fsd_build_precondition_frame(&pf);
        g_can->send(pf);
        last_precond_ms = now;
    }

    // ── BMS serial output ─────────────────────────────────────────────────────
    static uint32_t last_bms_ms = 0;
    if (g_state.bms_output && g_state.bms_seen &&
        (now - last_bms_ms) >= BMS_PRINT_MS) {
        float kw = g_state.pack_voltage_v * g_state.pack_current_a / 1000.0f;
        Serial.printf("[BMS] %.1fV  %.1fA  %.2fkW  SoC:%.1f%%  Temp:%d~%d°C\n",
            g_state.pack_voltage_v,
            g_state.pack_current_a,
            kw,
            g_state.soc_percent,
            (int)g_state.batt_temp_min_c,
            (int)g_state.batt_temp_max_c);
        last_bms_ms = now;
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
            "[STA] HW:%-6s FSD:%-4s NAG:%-10s OTA:%-3s "
            "Profile:%d  RX:%lu TX:%lu Err:%lu\n",
            hw_str,
            g_state.fsd_enabled     ? "ON"         : "wait",
            g_state.nag_suppressed  ? "suppressed"  : "active",
            g_state.tesla_ota_in_progress ? "YES"  : "no",
            g_state.speed_profile,
            (unsigned long)g_state.rx_count,
            (unsigned long)g_state.frames_modified,
            (unsigned long)g_state.crc_err_count);
        last_status_ms = now;
    }

    // ── Wiring sanity warning ─────────────────────────────────────────────────
    // Suppressed during a timer-wake probe: silence is the expected case there
    // and we'd drop straight back to sleep before the user could act on it.
    static uint32_t last_warn_ms = 0;
    bool warn_eligible = (g_state.rx_count == 0 && now > WIRING_WARN_MS);
#if defined(SLEEP_STRATEGY_TIMER)
    if (g_probe_mode) warn_eligible = false;
#endif
    if (warn_eligible && (now - last_warn_ms) >= 2000u) {
        Serial.println("[WARN] No CAN traffic after 5 s — check wiring");
        Serial.println("[WARN] Verify CAN-H on OBD pin 6, CAN-L on pin 14");
        last_warn_ms = now;
    }

    can_dump_tick(now);

#if defined(SLEEP_STRATEGY_EXT0) || defined(SLEEP_STRATEGY_TIMER)
    sleep_tick(now);
#endif

    // ── Web dashboard (after CAN to preserve CAN frame latency) ──────────────
    web_dashboard_update();

    update_led();
}
