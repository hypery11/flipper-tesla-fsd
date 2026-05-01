#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ── CAN frame (shared by all drivers) ────────────────────────────────────────
struct CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

// ── Hardware version ──────────────────────────────────────────────────────────
typedef enum {
    TeslaHW_Unknown = 0,
    TeslaHW_Legacy,   // HW1 / HW2
    TeslaHW_HW3,
    TeslaHW_HW4,
} TeslaHWVersion;

// ── Operation mode ────────────────────────────────────────────────────────────
typedef enum {
    OpMode_ListenOnly = 0,  // default on boot: no TX
    OpMode_Active,          // TX enabled
} OpMode;

// ── Full FSD state ────────────────────────────────────────────────────────────
struct FSDState {
    TeslaHWVersion hw_version;
    // User-set override. TeslaHW_Unknown = use auto-detect; any other value
    // pins hw_version to that HW and tells apply_detected_hw() to skip
    // auto-detect updates. Useful when 0x398 isn't on the tapped CAN bus
    // and the 0x399-fallback misclassifies the car (e.g. EU HW3 with ISA
    // active broadcasts 0x399 even though it isn't HW4).
    TeslaHWVersion hw_override;
    int            speed_profile;   // 0-4 depending on HW
    int            speed_offset;    // HW3 only, 0-100

    bool           fsd_enabled;     // true when car's UI has FSD selected (mux0)
    bool           nag_suppressed;  // true after first nag-killer echo sent

    uint32_t       frames_modified; // TX counter

    // ── Feature flags (runtime-toggleable) ───────────────────────────────────
    bool           force_fsd;               // bypass UI selection check
    bool           suppress_speed_chime;    // ISA chime suppress (HW4, 0x399)
    bool           emergency_vehicle_detect;// set bit59 in mux0 (HW4)
    bool           nag_killer;              // 0x370 counter+1 echo
    uint32_t       nag_echo_count;

    // ── Mode + diagnostics ────────────────────────────────────────────────────
    OpMode         op_mode;
    bool           tesla_ota_in_progress;   // pause TX during OTA
    bool           ota_ignore;              // user override: ignore OTA detection
                                            // (workaround for cars whose 0x318
                                            // byte-6 encoding doesn't match the
                                            // upstream OTA_IN_PROGRESS_RAW_VALUE)
    uint8_t        ota_raw_state;           // raw GTW_updateInProgress bits [1:0]
    uint8_t        ota_assert_count;        // consecutive "in-progress" samples
    uint8_t        ota_clear_count;         // consecutive "not in-progress" samples
    uint32_t       crc_err_count;           // CAN bus error counter
    uint32_t       rx_count;                // total frames seen (wiring check)
    uint32_t       seen_gtw_car_state;      // 0x318 seen count
    uint32_t       seen_gtw_car_config;     // 0x398 seen count
    uint32_t       seen_ap_control;         // 0x3FD seen count
    uint32_t       seen_follow_dist;        // 0x3F8 seen count (stalk)
    uint32_t       seen_bms_hv;             // 0x132 seen count
    uint32_t       seen_bms_soc;            // 0x292 seen count
    uint32_t       seen_bms_thermal;        // 0x312 seen count

    // ── BMS read-only sniff ───────────────────────────────────────────────────
    bool           bms_output;       // print BMS data to serial
    bool           bms_seen;
    float          pack_voltage_v;
    float          pack_current_a;
    float          soc_percent;
    int8_t         batt_temp_min_c;
    int8_t         batt_temp_max_c;
    // 12 V auxiliary bus (Chassis-CAN 0x2B5). Useful diagnostic — a low LV
    // bus voltage is a common Tesla failure mode (dead 12V battery).
    bool           lv_bus_seen;
    float          lv_bus_voltage_v;
    float          lv_bus_current_a;

    // ── Vehicle dynamics (read-only, parsed from Party CAN) ─────────────────
    bool           speed_seen;
    float          vehicle_speed_kph;     // 0x257 DI_vehicleSpeed (12-bit, 0.08, -40)
    uint8_t        ui_speed;              // 0x257 DI_uiSpeed (display value)
    bool           steering_seen;
    float          steering_angle_deg;    // 0x129 (signed 16-bit, ×0.1)
    bool           torque_seen;
    float          motor_torque_nm;       // 0x108 DI_torque1 (13-bit, 0.25, -750)
    bool           brake_seen;
    bool           driver_brake_applied;  // 0x145 ESP_driverBrakeApply

    // ── DAS_status full (0x39B) — extends das_hands_on_state below ──────────
    uint8_t        das_lane_change;       // 5-bit, lane change state
    uint8_t        das_side_coll_warn;    // 2-bit, side collision warning (blind spot)
    uint8_t        das_side_coll_avoid;   // 2-bit, side collision avoid
    uint8_t        das_fcw;               // 2-bit, forward collision warning
    uint8_t        das_vision_speed_lim;  // 5-bit, ×5 = kph (vision-based limit)

    // ── Raw frame snapshots (for in-car bit-position debugging) ─────────────
    // Last seen DLC + bytes for IDs whose parsing is firmware-version-fragile.
    // Surfaced on the dashboard as hex; press the relevant control on the car
    // (brake pedal, turn wheel) and watch which byte changes to identify the
    // correct bit position.
    uint8_t        raw_145_dlc;
    uint8_t        raw_145_bytes[8];
    uint8_t        raw_39b_dlc;
    uint8_t        raw_39b_bytes[8];
    uint8_t        raw_129_dlc;
    uint8_t        raw_129_bytes[8];

    // ── CAN serial trace (for finding unknown signal positions) ─────────────
    // When can_trace is true, every frame whose bytes differ from the last
    // capture for its ID is printed to serial as
    //   [TRACE] 0x145 dlc=8: 41 02 00 00 00 00 00 03
    // The last-bytes table below is kept regardless of trace state so that
    // turning the trace on doesn't briefly print every steady-state frame.
    bool           can_trace;              // runtime-only, default off
    uint16_t       trace_count;            // number of unique IDs seen
    #define TRACE_MAX 80
    uint16_t       trace_ids[TRACE_MAX];
    uint8_t        trace_dlc[TRACE_MAX];
    uint8_t        trace_bytes[TRACE_MAX][8];

    // ── Precondition trigger ──────────────────────────────────────────────────
    bool           precondition;     // periodically inject 0x082

    // ── Deep sleep ────────────────────────────────────────────────────────────
    uint32_t       sleep_idle_ms;    // CAN silence before entering deep sleep

    // ── WiFi ──────────────────────────────────────────────────────────────────
    char           wifi_ssid[33];    // max 32 chars + null
    char           wifi_pass[65];    // max 64 chars + null
    bool           wifi_hidden;

    // ── TLSSC Restore (0x331 DAS config spoof) ──────────────────────────────
    bool           tlssc_restore;
    uint32_t       tlssc_restore_count;

    // ── DAS_autopilot readback (parsed from 0x331 byte[0]) ──────────────────
    // byte[0] lower 6 bits encodes two 3-bit tiers (0..4):
    //   bits 5:3 = DAS_autopilot, bits 2:0 = DAS_autopilotBase
    // Tier enum: 0=NONE 1=HIGHWAY 2=ENHANCED 3=SELF_DRIVING 4=BASIC
    bool           das_ap_seen;
    uint8_t        das_autopilot;       // bits 5:3
    uint8_t        das_autopilot_base;  // bits 2:0
    uint32_t       seen_das_ap_config;  // 0x331 frame counter

    // ── DAS status (0x39B) — nag killer gating ───────────────────────────────
    // 0=NOT_REQD, 8=SUSPENDED — both mean DAS is satisfied, skip echo.
    // das_seen starts false; if 0x39B is absent from the tapped bus the nag
    // killer falls back to EPAS-level-only gating (conservative echo).
    bool           das_seen;
    uint8_t        das_hands_on_state;
};

// ── API ───────────────────────────────────────────────────────────────────────

/** Initialise state with safe defaults for a given HW version. */
void fsd_state_init(FSDState *state, TeslaHWVersion hw);

/** Update state for a newly detected HW version (preserves all settings). */
void fsd_apply_hw_version(FSDState *state, TeslaHWVersion hw);

/** Returns true if current state allows transmitting CAN frames. */
bool fsd_can_transmit(const FSDState *state);

/** Read GTW_carConfig (0x398) to detect HW version.
 *  Returns TeslaHW_Unknown if frame is not 0x398 or version unrecognised. */
TeslaHWVersion fsd_detect_hw_version(const CanFrame *frame);

/** Parse GTW_carState (0x318) — updates tesla_ota_in_progress. */
void fsd_handle_gtw_car_state(FSDState *state, const CanFrame *frame);

/** Parse DAS_followDistance (0x3F8) — updates speed_profile from stalk. */
void fsd_handle_follow_distance(FSDState *state, const CanFrame *frame);

/** Modify DAS_autopilotControl (0x3FD) for HW3/HW4.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_autopilot_frame(FSDState *state, CanFrame *frame);

/** Parse STW_ACTN_RQ (0x045) for Legacy stalk position → speed_profile. */
void fsd_handle_legacy_stalk(FSDState *state, const CanFrame *frame);

/** Modify DAS_autopilot (0x3EE) for Legacy/HW1/HW2.
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_legacy_autopilot(FSDState *state, CanFrame *frame);

/** Modify ISA speed limit frame (0x399) to suppress speed chime (HW4).
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_isa_speed_chime(CanFrame *frame);

/** Build an echo of EPAS3P_sysStatus (0x370) with counter+1 and handsOnLevel=1.
 *  Writes result into *out.  Returns true if echo should be sent. */
bool fsd_handle_nag_killer(FSDState *state, const CanFrame *frame, CanFrame *out);

/** Parse BMS_hvBusStatus (0x132) — updates pack_voltage_v / pack_current_a. */
void fsd_handle_bms_hv(FSDState *state, const CanFrame *frame);

/** Parse BMS_socStatus (0x292) — updates soc_percent. */
void fsd_handle_bms_soc(FSDState *state, const CanFrame *frame);

/** Parse BMS_thermalStatus (0x312) — updates batt_temp_min/max_c. */
void fsd_handle_bms_thermal(FSDState *state, const CanFrame *frame);

/** Build a UI_tripPlanning (0x082) frame to trigger active battery heating. */
void fsd_build_precondition_frame(CanFrame *frame);

/** Handle CAN ID 0x331 — TLSSC Restore via DAS config spoof.
 *  Overwrites byte[0] lower 6 bits to 0x1B (SELF_DRIVING).
 *  Returns true if frame was modified and should be re-sent. */
bool fsd_handle_tlssc_restore(FSDState *state, CanFrame *frame);

/** Parse DAS_status (0x39B) — updates das_hands_on_state plus lane change,
 *  side-collision warn/avoid, FCW and vision speed limit fields. */
void fsd_handle_das_status(FSDState *state, const CanFrame *frame);

/** Parse DAS config (0x331) — updates das_autopilot / das_autopilot_base
 *  tier readback fields (0=NONE 1=HIGHWAY 2=ENHANCED 3=SELF_DRIVING 4=BASIC). */
void fsd_handle_das_ap_config(FSDState *state, const CanFrame *frame);

/** Parse DI_speed (0x257) — vehicle speed in kph + UI display value. */
void fsd_handle_di_speed(FSDState *state, const CanFrame *frame);

/** Parse ESP_status (0x145) — driver brake apply flag (Party CAN). */
void fsd_handle_esp_status(FSDState *state, const CanFrame *frame);


/** Parse DI_torque (0x108) — drive motor torque in Nm. */
void fsd_handle_di_torque(FSDState *state, const CanFrame *frame);

/** Parse SCCM_steeringAngleSensor (0x129) — steering angle in degrees. */
void fsd_handle_steering_angle(FSDState *state, const CanFrame *frame);

/** Parse 0x420 — Chassis-CAN battery status fallback. byte 2 = SoC %.
 *  Empirically identified on 2026.8.3 EU HW3 where the standard BMS frames
 *  (0x132/0x292/0x312) are not broadcast on Chassis CAN. */
void fsd_handle_batt_status_chassis(FSDState *state, const CanFrame *frame);

/** Parse 0x239 — Chassis-CAN battery temperature.
 *  byte 5 × 0.5 − 40 = battery temp °C (standard Tesla cell-temp encoding). */
void fsd_handle_batt_temp(FSDState *state, const CanFrame *frame);

/** Parse 0x2B5 — Chassis-CAN DC bus status. Carries LV (12V) bus voltage +
 *  current and the HV pack voltage. Empirically identified on Chassis CAN. */
void fsd_handle_dc_bus(FSDState *state, const CanFrame *frame);
