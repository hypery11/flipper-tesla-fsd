/*
 * fsd_handler.cpp
 *
 * CAN frame manipulation logic for Tesla FSD unlock.
 * Ported from hypery11/flipper-tesla-fsd  fsd_logic/fsd_handler.c
 *
 * All bit operations, mux dispatch, speed profile mapping, and checksum
 * calculations are kept bit-for-bit identical to the upstream Flipper Zero
 * implementation.
 */

#include "fsd_handler.h"
#include <string.h>

// ── Internal helpers ──────────────────────────────────────────────────────────

static void set_bit(CanFrame *frame, int bit, bool value) {
    if (bit < 0 || bit >= 64) return;
    int byte_idx = bit / 8;
    int bit_idx  = bit % 8;
    uint8_t mask = (uint8_t)(1U << bit_idx);
    if (value)
        frame->data[byte_idx] |= mask;
    else
        frame->data[byte_idx] &= (uint8_t)(~mask);
}

static uint8_t read_mux_id(const CanFrame *frame) {
    // MUX ID is the lower 3 bits of byte 0
    return frame->data[0] & 0x07;
}

static bool is_fsd_selected(const CanFrame *frame, bool force_fsd) {
    if (force_fsd) return true;
    if (frame->dlc < 5) return false;
    // DAS_autopilotControl byte 4 bits [7:6] = UI "FSD selected" flag (bit 38 in the 64-bit data
    // field).  Note: bit 46 is the *output* FSD-activation bit written to the modified frame —
    // a different field at byte 5 bit 6.
    return (frame->data[4] >> 6) & 0x01;
}

// ── State init ────────────────────────────────────────────────────────────────

void fsd_state_init(FSDState *state, TeslaHWVersion hw) {
    memset(state, 0, sizeof(FSDState));
    fsd_apply_hw_version(state, hw);
    state->op_mode    = OpMode_ListenOnly;  // safe default — never TX on boot

    // Feature flags: nag killer and chime suppress default ON; others OFF
    state->nag_killer           = true;
    state->suppress_speed_chime = true;
    state->emergency_vehicle_detect = false;
    state->force_fsd            = false;
    state->bms_output           = false;
    state->sleep_idle_ms        = SLEEP_IDLE_MS;
    state->hw_override          = TeslaHW_Unknown;  // 0 = auto-detect
    state->ota_ignore           = false;

    // Build-time overridable via WIFI_DEFAULT_SSID / WIFI_DEFAULT_PASS in
    // platformio.ini (or a gitignored platformio_local.ini) so personal
    // credentials don't ship in the public repo.
    #ifndef WIFI_DEFAULT_SSID
    #define WIFI_DEFAULT_SSID "Tesla-FSD"
    #endif
    #ifndef WIFI_DEFAULT_PASS
    #define WIFI_DEFAULT_PASS "12345678"
    #endif
    strncpy(state->wifi_ssid, WIFI_DEFAULT_SSID, sizeof(state->wifi_ssid));
    strncpy(state->wifi_pass, WIFI_DEFAULT_PASS, sizeof(state->wifi_pass));
    state->wifi_hidden = false;
}

void fsd_apply_hw_version(FSDState *state, TeslaHWVersion hw) {
    state->hw_version = hw;
    // Default speed profile per HW version
    if (hw == TeslaHW_HW4)
        state->speed_profile = 4;
    else if (hw == TeslaHW_Legacy)
        state->speed_profile = 1;
    else
        state->speed_profile = 2;
}

// ── Transmit gate ─────────────────────────────────────────────────────────────

bool fsd_can_transmit(const FSDState *state) {
    if (state->op_mode == OpMode_ListenOnly) return false;
    if (state->tesla_ota_in_progress && !state->ota_ignore) return false;
    return true;
}

// ── HW version detection from GTW_carConfig (0x398) ──────────────────────────

TeslaHWVersion fsd_detect_hw_version(const CanFrame *frame) {
    if (frame->id != CAN_ID_GTW_CAR_CONFIG) return TeslaHW_Unknown;
    // DAS_HWversion field: bits 7:6 of byte 0  (das_hw)
    uint8_t das_hw = (frame->data[0] >> 6) & 0x03;
    switch (das_hw) {
        case 0:
        case 1:  return TeslaHW_Legacy;   // HW1/HW2/EAP retrofit — uses 0x3EE/0x045
        case 2:  return TeslaHW_HW3;
        case 3:  return TeslaHW_HW4;
        default: return TeslaHW_Unknown;
    }
}

// ── OTA detection from GTW_carState (0x318) ───────────────────────────────────

void fsd_handle_gtw_car_state(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 7) return;
    // GTW_updateInProgress: bits 1:0 of byte 6.
    // Filter transient / incompatible values to avoid false positives.
    uint8_t raw = frame->data[6] & 0x03u;
    state->ota_raw_state = raw;

    bool in_progress = (raw == OTA_IN_PROGRESS_RAW_VALUE);
    if (in_progress) {
        if (state->ota_assert_count < 255u) state->ota_assert_count++;
        state->ota_clear_count = 0;
        if (state->ota_assert_count >= OTA_ASSERT_FRAMES)
            state->tesla_ota_in_progress = true;
    } else {
        if (state->ota_clear_count < 255u) state->ota_clear_count++;
        state->ota_assert_count = 0;
        if (state->ota_clear_count >= OTA_CLEAR_FRAMES)
            state->tesla_ota_in_progress = false;
    }
}

// ── Follow distance → speed profile (DAS_followDistance 0x3F8) ───────────────

void fsd_handle_follow_distance(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 6) return;
    // Follow distance stalk position: bits 7:5 of byte 5
    uint8_t fd = (frame->data[5] & 0xE0) >> 5;

    if (state->hw_version == TeslaHW_HW3) {
        // HW3: 3 levels  (fd 1→profile 2, 2→1, 3→0)
        switch (fd) {
            case 1: state->speed_profile = 2; break;
            case 2: state->speed_profile = 1; break;
            case 3: state->speed_profile = 0; break;
            default: break;
        }
    } else {
        // HW4: 5 levels  (fd 1→3, 2→2, 3→1, 4→0, 5→4)
        switch (fd) {
            case 1: state->speed_profile = 3; break;
            case 2: state->speed_profile = 2; break;
            case 3: state->speed_profile = 1; break;
            case 4: state->speed_profile = 0; break;
            case 5: state->speed_profile = 4; break;
            default: break;
        }
    }
}

// ── HW3/HW4 autopilot control (DAS_autopilotControl 0x3FD) ───────────────────

bool fsd_handle_autopilot_frame(FSDState *state, CanFrame *frame) {
    if (frame->dlc < 8) return false;
    // Only process known HW versions to avoid corrupting frames for HW_Unknown
    if (state->hw_version != TeslaHW_HW3 && state->hw_version != TeslaHW_HW4)
        return false;

    uint8_t mux     = read_mux_id(frame);
    bool    fsd_ui  = is_fsd_selected(frame, state->force_fsd);
    bool    modified = false;

    // mux 0 is the authoritative "is FSD requested" mux
    if (mux == 0) state->fsd_enabled = fsd_ui;

    if (state->hw_version == TeslaHW_HW3) {
        // ── HW3 ──────────────────────────────────────────────────────────────
        if (mux == 0 && state->fsd_enabled) {
            // Compute speed offset from current speed signal (bits 6:1 of byte 3)
            int raw    = (int)((frame->data[3] >> 1) & 0x3F) - 30;
            int offset = raw * 5;
            if (offset < 0)   offset = 0;
            if (offset > 100) offset = 100;
            state->speed_offset = offset;

            // Activate FSD: set bit 46
            set_bit(frame, 46, true);

            // Write speed profile into bits 2:1 of byte 6
            frame->data[6] &= ~0x06u;
            frame->data[6] |= (uint8_t)((state->speed_profile & 0x03) << 1);
            modified = true;
        }
        if (mux == 1) {
            // Nag suppression via bit 19 (clear = no hands-on-wheel request)
            set_bit(frame, 19, false);
            state->nag_suppressed = true;
            modified = true;
        }
        if (mux == 2 && state->fsd_enabled) {
            // Write speed offset into bits 7:6 of byte 0 and bits 5:0 of byte 1
            frame->data[0] &= ~0xC0u;
            frame->data[1] &= ~0x3Fu;
            frame->data[0] |= (uint8_t)((state->speed_offset & 0x03) << 6);
            frame->data[1] |= (uint8_t)(state->speed_offset >> 2);
            modified = true;
        }
    } else {
        // ── HW4 ──────────────────────────────────────────────────────────────
        if (mux == 0 && state->fsd_enabled) {
            set_bit(frame, 46, true);   // FSD activation
            set_bit(frame, 60, true);   // HW4 additional FSD bit
            if (state->emergency_vehicle_detect)
                set_bit(frame, 59, true);  // emergency vehicle detection
            modified = true;
        }
        if (mux == 1) {
            set_bit(frame, 19, false);  // clear hands-on-wheel nag
            set_bit(frame, 47, true);   // HW4 nag-suppression confirmation bit
            state->nag_suppressed = true;
            modified = true;
        }
        if (mux == 2) {
            // Write speed profile into bits 6:4 of byte 7
            frame->data[7] &= ~(uint8_t)(0x07u << 5);
            frame->data[7] |=  (uint8_t)((state->speed_profile & 0x07u) << 5);
            modified = true;
        }
    }

    if (modified) state->frames_modified++;
    return modified;
}

// ── Legacy autopilot (DAS_autopilot 0x3EE) ───────────────────────────────────

void fsd_handle_legacy_stalk(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 2) return;
    // STW_ACTN_RQ: stalk position encoded in bits 7:5 of byte 1
    // 0x00=Pos1, 0x21=Pos2, 0x42=Pos3, 0x64=Pos4, 0x85=Pos5, 0xA6=Pos6, 0xC8=Pos7
    uint8_t pos = frame->data[1] >> 5;
    if (pos <= 1)
        state->speed_profile = 2;
    else if (pos == 2)
        state->speed_profile = 1;
    else
        state->speed_profile = 0;
}

bool fsd_handle_legacy_autopilot(FSDState *state, CanFrame *frame) {
    if (frame->dlc < 8) return false;

    uint8_t mux    = read_mux_id(frame);
    bool    fsd_ui = is_fsd_selected(frame, state->force_fsd);
    bool    modified = false;

    if (mux == 0) state->fsd_enabled = fsd_ui;

    if (mux == 0 && state->fsd_enabled) {
        set_bit(frame, 46, true);
        // Speed profile in bits 2:1 of byte 6 (same encoding as HW3)
        frame->data[6] &= ~0x06u;
        frame->data[6] |= (uint8_t)((state->speed_profile & 0x03) << 1);
        modified = true;
    }
    if (mux == 1) {
        set_bit(frame, 19, false);
        state->nag_suppressed = true;
        modified = true;
    }

    if (modified) state->frames_modified++;
    return modified;
}

// ── ISA speed chime suppress (0x399, HW4 only) ───────────────────────────────

bool fsd_handle_isa_speed_chime(CanFrame *frame) {
    if (frame->dlc < 8) return false;
    // Set "ISA_speedLimitSoundActive" flag: bit 5 of byte 1
    frame->data[1] |= 0x20u;
    // Recalculate Tesla checksum: sum(byte0..6) + low(CAN_ID) + high(CAN_ID)
    // CAN_ID_ISA_SPEED = 0x399 → low=0x99, high=0x03
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++)
        sum += frame->data[i];
    sum += (uint8_t)(CAN_ID_ISA_SPEED & 0xFFu) + (uint8_t)(CAN_ID_ISA_SPEED >> 8);
    frame->data[7] = sum;
    return true;
}

// ── NAG killer: counter+1 echo of EPAS3P_sysStatus (0x370) ──────────────────
//
// When handsOnLevel == 0 (nag imminent) or 3 (escalated alarm), we send a
// spoofed EPAS frame with handsOnLevel=1 and counter+1 before the real frame
// reaches the DAS.  The DAS sees "hands on" and drops the nag.
//
// DAS-aware gating: also checks das_hands_on_state from 0x39B.  States 0
// (NOT_REQD) and 8 (SUSPENDED) mean DAS is already satisfied — skip the echo
// to avoid ~25 spurious frames/sec on the bus.  If 0x39B has never been seen
// (das_seen==false), we echo conservatively based on EPAS level alone.
//
// Organic torque: torsionBarTorque uses a xorshift32 random walk [1.00–2.40 Nm]
// with brief grip pulses [3.10–3.30 Nm] every 5–9 s.  A flat signal for 30+
// minutes is a statistical impossibility from a real hand and is a known
// telemetry detection vector.
//
// Checksum: byte7 = (sum(byte0..6) + 0x70 + 0x03) & 0xFF  (CAN ID 0x370 split)

static uint32_t nag_prng_state       = 0xDEADBEEFu;
static int16_t  nag_torq_walk        = 2230;   // raw init ≈ 1.80 Nm
static uint8_t  nag_exc_frames       = 0;
static uint16_t nag_frames_until_exc = 175;

static uint32_t nag_xorshift32() {
    uint32_t x = nag_prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    nag_prng_state = x;
    return x;
}

bool fsd_handle_nag_killer(FSDState *state, const CanFrame *frame, CanFrame *out) {
    if (frame->dlc < 8)     return false;
    if (!state->nag_killer) return false;

    // EPAS handsOnLevel: bits 7:6 of byte 4.  Skip only when level==1 (hands OK).
    uint8_t hands_on = (frame->data[4] >> 6) & 0x03u;
    if (hands_on == 1) return false;

    // DAS-aware gating — skip echo when DAS itself is satisfied.
    if (state->das_seen) {
        uint8_t das = state->das_hands_on_state;
        if (das == 0 || das == 8) return false;
    }

    // Organic torque random walk
    int16_t torq;
    if (nag_exc_frames > 0) {
        // Grip pulse: ~3.20 Nm ± noise
        torq = 2350 + (int16_t)((int)(nag_xorshift32() % 41u) - 20);
        nag_exc_frames--;
    } else {
        int16_t step = (int16_t)((int)(nag_xorshift32() % 31u) - 15);
        nag_torq_walk += step;
        if (nag_torq_walk < 2150) nag_torq_walk = 2150;  // min ~1.00 Nm
        if (nag_torq_walk > 2290) nag_torq_walk = 2290;  // max ~2.40 Nm
        torq = nag_torq_walk;
        if (nag_frames_until_exc > 0) {
            nag_frames_until_exc--;
        } else {
            nag_exc_frames       = (uint8_t)(3u + (nag_xorshift32() % 3u));
            nag_frames_until_exc = (uint16_t)(125u + (nag_xorshift32() % 100u));
        }
    }

    out->id  = CAN_ID_EPAS_STATUS;
    out->dlc = 8;

    out->data[0] = frame->data[0];
    out->data[1] = frame->data[1];
    out->data[2] = (frame->data[2] & 0xF0u) | (uint8_t)((torq >> 8) & 0x0Fu);
    out->data[3] = (uint8_t)(torq & 0xFFu);
    out->data[4] = (frame->data[4] & ~0xC0u) | 0x40u;  // handsOnLevel = 1
    out->data[5] = frame->data[5];

    // counter+1: lower nibble of byte 6
    uint8_t cnt = (frame->data[6] & 0x0Fu);
    cnt = (cnt + 1u) & 0x0Fu;
    out->data[6] = (frame->data[6] & 0xF0u) | cnt;

    // Checksum
    uint16_t sum = 0;
    for (int i = 0; i < 7; i++)
        sum += out->data[i];
    sum += (CAN_ID_EPAS_STATUS & 0xFFu) + (CAN_ID_EPAS_STATUS >> 8);
    out->data[7] = (uint8_t)(sum & 0xFFu);

    state->nag_echo_count++;
    state->nag_suppressed = true;
    return true;
}

// ── BMS read-only parsers ─────────────────────────────────────────────────────

void fsd_handle_bms_hv(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 4) return;
    // Voltage: uint16 little-endian bytes 1:0, LSB = 0.01 V
    uint16_t raw_v = ((uint16_t)frame->data[1] << 8) | frame->data[0];
    // Current: int16 little-endian bytes 3:2, LSB = 0.1 A (signed)
    int16_t  raw_i = (int16_t)(((uint16_t)frame->data[3] << 8) | frame->data[2]);
    state->pack_voltage_v = raw_v * 0.01f;
    state->pack_current_a = raw_i * 0.1f;
    state->bms_seen = true;
}

void fsd_handle_bms_soc(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 2) return;
    // SoC: 10-bit little-endian (bits 9:0 across bytes 1:0), LSB = 0.1 %
    uint16_t raw = ((uint16_t)(frame->data[1] & 0x03u) << 8) | frame->data[0];
    state->soc_percent = raw * 0.1f;
    state->bms_seen = true;
}

void fsd_handle_bms_thermal(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 6) return;
    // Temperatures: raw byte − 40 = °C
    state->batt_temp_min_c = (int8_t)((int)frame->data[4] - 40);
    state->batt_temp_max_c = (int8_t)((int)frame->data[5] - 40);
    state->bms_seen = true;
}

// ── Precondition trigger ──────────────────────────────────────────────────────

void fsd_build_precondition_frame(CanFrame *frame) {
    memset(frame, 0, sizeof(CanFrame));
    frame->id  = CAN_ID_TRIP_PLANNING;
    frame->dlc = 8;
    // byte0: bit0 = tripPlanningActive, bit2 = requestActiveBatteryHeating
    frame->data[0] = 0x05u;
}

// ── TLSSC Restore (0x331) ─────────────────────────────────────────────────────

bool fsd_handle_tlssc_restore(FSDState *state, CanFrame *frame) {
    if (!state->tlssc_restore) return false;
    if (frame->dlc < 1) return false;

    uint8_t original = frame->data[0];
    uint8_t modified = (original & 0xC0u) | 0x1Bu;

    if (modified == original) return false;

    frame->data[0] = modified;
    state->tlssc_restore_count++;
    return true;
}

// ── DAS status (0x39B) — nag killer gating + diagnostics readback ───────────

void fsd_handle_das_status(FSDState *state, const CanFrame *frame) {
    if (frame->dlc == 0) return;
    // Mark seen so the dashboard knows the ID is on this bus, even if a
    // particular field's bytes aren't present.
    state->das_seen = true;
    state->raw_39b_dlc = frame->dlc;
    for (uint8_t i = 0; i < frame->dlc && i < 8; i++)
        state->raw_39b_bytes[i] = frame->data[i];
    if (frame->dlc < 6) return;
    // DAS_autopilotHandsOnState: bit42|4 LE → byte5 bits[5:2]
    state->das_hands_on_state = (frame->data[5] >> 2) & 0x0Fu;
    // DAS_autoLaneChangeState: bit46|5 LE → byte5 bits[7:6] + byte6 bits[2:0]
    if (frame->dlc >= 7) {
        state->das_lane_change = ((frame->data[5] >> 6) & 0x03u) |
                                 ((frame->data[6] & 0x07u) << 2);
    }
    // DAS_sideCollisionWarning: bit32|2 → byte4 bits[1:0]
    // DAS_sideCollisionAvoid:   bit30|2 → byte3 bits[7:6]
    if (frame->dlc >= 5) {
        state->das_side_coll_warn  = frame->data[4] & 0x03u;
        state->das_side_coll_avoid = (frame->data[3] >> 6) & 0x03u;
    }
    // DAS_forwardCollisionWarning: bit22|2 → byte2 bits[7:6]
    // DAS_visionOnlySpeedLimit:    bit16|5 → byte2 bits[4:0], ×5 = kph
    if (frame->dlc >= 3) {
        state->das_fcw              = (frame->data[2] >> 6) & 0x03u;
        state->das_vision_speed_lim = frame->data[2] & 0x1Fu;
    }
}

// ── DAS_autopilot config readback (0x331) ────────────────────────────────────
// byte[0] lower 6 bits encodes two 3-bit tiers (DAS_autopilot / Base).
// Tier enum: 0=NONE 1=HIGHWAY 2=ENHANCED 3=SELF_DRIVING 4=BASIC.
void fsd_handle_das_ap_config(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 1) return;
    uint8_t b0 = frame->data[0];
    state->das_autopilot      = (b0 >> 3) & 0x07u;
    state->das_autopilot_base = b0 & 0x07u;
    state->das_ap_seen        = true;
}

// ── DI_speed (0x257) — vehicle speed ─────────────────────────────────────────
// DI_vehicleSpeed: 12-bit LE starting at bit 12 → byte1 high nibble + byte2
// scale 0.08, offset -40, units kph.
// DI_uiSpeed: bit24|8 → byte 3 (display speed, integer kph or mph).
void fsd_handle_di_speed(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 4) return;
    uint16_t raw = (((uint16_t)frame->data[2]) << 4) | ((frame->data[1] >> 4) & 0x0Fu);
    float v = (float)raw * 0.08f - 40.0f;
    if (v < 0.0f) v = 0.0f;
    state->vehicle_speed_kph = v;
    state->ui_speed = frame->data[3];
    state->speed_seen = true;
}

// ── ESP_v118 (0x145) — driver brake pedal ────────────────────────────────────
// opendbc tesla_model3_party.dbc:
//   ESP_brakePedalPressed: 19|1@1+ → byte 2, bit 3 (LE)
// (The Flipper code reads byte 3 bits [6:5]; that's a different bit on this
//  firmware version and reads as constantly non-zero, so we use the opendbc
//  position here.)
void fsd_handle_esp_status(FSDState *state, const CanFrame *frame) {
    if (frame->dlc == 0) return;
    state->raw_145_dlc = frame->dlc;
    for (uint8_t i = 0; i < frame->dlc && i < 8; i++)
        state->raw_145_bytes[i] = frame->data[i];
    if (frame->dlc < 3) return;
    // Note: this position (byte 2 bit 3 per opendbc) is observed to NOT toggle
    // on EU HW3 firmware via Chassis CAN — the Chassis-CAN brake bit lives in
    // 0x102 instead (see fsd_handle_vcleft_brake). We only set brake_applied
    // here when the brake_seen flag isn't already being driven by 0x102.
    if (!state->brake_seen) {
        state->driver_brake_applied = ((frame->data[2] >> 3) & 0x01u) != 0;
        state->brake_seen = true;
    }
}

// (NOTE: 0x102 byte 4 bit 1 was tested as a brake candidate from a 3-tap
//  capture but turned out to flicker independently of brake input — false
//  correlation. Brake on Chassis CAN appears to live somewhere we haven't
//  identified yet; needs more reverse engineering. The 0x145 ESP_status
//  parser above remains the Party-CAN brake source per opendbc.)

// ── DI_torque1 (0x108) — drive motor torque ──────────────────────────────────
// opendbc tesla_model3_party.dbc:
//   DI_torqueMotor: 21|13@1+ (0.222656, -750) — actual motor output, Nm
// LE 13-bit unsigned starting at bit 21:
//   byte[2] bits [7:5] = bits 21..23 (low 3 bits)
//   byte[3] bits [7:0] = bits 24..31 (mid 8 bits)
//   byte[4] bits [1:0] = bits 32..33 (high 2 bits)
// (The Flipper code read DI_torqueDriver at bit 0 with scale 0.25 — on this
//  firmware that value moves around unrelated to actual motor torque.)
void fsd_handle_di_torque(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 5) return;
    uint16_t raw = (uint16_t)((frame->data[2] >> 5) & 0x07u)
                 | ((uint16_t)frame->data[3] << 3)
                 | ((uint16_t)(frame->data[4] & 0x03u) << 11);
    state->motor_torque_nm = (float)raw * 0.222656f - 750.0f;
    state->torque_seen = true;
}

// ── SCCM_steeringAngleSensor (0x129) — steering wheel angle ──────────────────
// opendbc tesla_model3_party.dbc:
//   SCCM_steeringAngleSensor: 16|14@1+ (0.1, -819.2)
// LE 14-bit unsigned starting at bit 16 = byte 2, with offset:
//   byte[2] = bits 16..23 (low 8 bits)
//   byte[3] bits [5:0] = bits 24..29 (high 6 bits)
// (The Flipper code read raw int16 from bytes 0-1, which on this firmware is
//  a different field that jumps unrelated to wheel position.)
void fsd_handle_steering_angle(FSDState *state, const CanFrame *frame) {
    if (frame->dlc == 0) return;
    state->steering_seen = true;
    state->raw_129_dlc = frame->dlc;
    for (uint8_t i = 0; i < frame->dlc && i < 8; i++)
        state->raw_129_bytes[i] = frame->data[i];
    if (frame->dlc < 4) return;
    uint16_t raw = (uint16_t)frame->data[2] | ((uint16_t)(frame->data[3] & 0x3Fu) << 8);
    state->steering_angle_deg = (float)raw * 0.1f - 819.2f;
}

// ── 0x420 — Chassis-CAN battery status fallback ─────────────────────────────
// Reverse-engineered against a 2026.8.3 EU HW3 car (Chassis CAN tap):
//   byte 0 = battery temp °C × 2 (tentative — verified at 0x28 = 40 raw,
//            so 20°C, vs cluster reading of ~21.5°C — within sensor delta).
//   byte 2 = SoC % (verified at 64% with byte 2 = 0x40).
//   byte 3 = stable value, meaning unknown (verified != charge target).
// Standard BMS frames (0x132/0x292/0x312) aren't broadcast on Chassis CAN
// here. Pack voltage/current still unavailable without UDS query/response.
void fsd_handle_batt_status_chassis(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 3) return;
    state->soc_percent      = (float)frame->data[2];
    state->bms_seen         = true;
}

// ── 0x239 — Chassis-CAN battery temperature ──────────────────────────────────
// Low-rate broadcast. byte 5 × 0.5 − 40 = battery temperature °C using the
// standard Tesla cell-temp encoding. Verified on live data against a
// 22°C-cluster reading. byte 2 also decodes in the temp range but the
// values don't track with the actual battery temp — likely some other
// field that just happens to fall in the same numeric range.
void fsd_handle_batt_temp(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 6) return;
    int t = (int)frame->data[5] / 2 - 40;
    state->batt_temp_min_c = (int8_t)t;
    state->batt_temp_max_c = (int8_t)t;
    state->bms_seen        = true;
}

// ── 0x2B5 — Chassis-CAN DC bus status ────────────────────────────────────────
// Reverse-engineered against a 2026.8.3 EU HW3 car (Chassis CAN tap),
// values cross-verified against enhauto Commander:
//   bytes 0-1 LE  × 0.01 = LV (12V) bus voltage  (e.g. 1580 = 15.80 V)
//   bytes 2-3 LE  × 0.1  = HV pack voltage       (e.g. 3785 = 378.5 V)
//   byte 4        × 0.1  = LV bus current (A)    (e.g. 0xC8 = 20.0 A)
// Populates pack_voltage_v (HV) and the new lv_bus_* fields. HV pack
// current isn't in this message; left as 0.
void fsd_handle_dc_bus(FSDState *state, const CanFrame *frame) {
    if (frame->dlc < 5) return;
    uint16_t lv_raw = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
    uint16_t hv_raw = (uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8);
    state->lv_bus_voltage_v = (float)lv_raw * 0.01f;
    state->lv_bus_current_a = (float)frame->data[4] * 0.1f;
    state->pack_voltage_v   = (float)hv_raw * 0.1f;
    state->lv_bus_seen      = true;
    state->bms_seen         = true;
}
