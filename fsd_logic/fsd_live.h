#pragma once
/*
 * fsd_live.h — read-only live values for the Flipper "Live Data" screen:
 * drive inverter currents, wheel speeds, drive-unit oil pumps and the pack.
 * Header-only and free of FSDState / platform code, like fsd_tpms.h. It only
 * decodes frames the car sends; it builds none.
 *
 * Layouts (all little-endian, Tesla bit numbering bit n = byte n/8, bit n%8):
 *
 *  Drive inverters, one per motor. IDs from Tesla's own S/X dictionary
 *  (tesla-can-explorer, Model S/X 2026.2 AMD = refreshed cars) and the
 *  Model 3/Y DBC (joshwardell/model3dbc):
 *                       motorStatus           torque
 *    front     DIF      0x1A5                 0x186
 *    rear      DIR      0x7EC S/X, 0x126 3/Y  0x108
 *    rear-left DIREL    0x7EF S/X, 0x127 3/Y  0x188       (Plaid)
 *    rear-rgt  DIRER    0x12A                 0x10A       (Plaid)
 *  motorStatus: *_motorCurrent bit 11|11 x1 A (unsigned, magnitude).
 *  torque:      *_torqueActual bit 27|13 signed x2 Nm,
 *               *_axleSpeed    bit 40|16 signed x0.1 RPM.
 *  The bit layout is from the 3/Y DBC; the S/X dictionary lists the same
 *  signal set for these frames but no bit positions. Unverified on S/X.
 *
 *  ESP_wheelSpeeds 0x175 (opendbc tesla_model3_party BO_373, also in
 *  esp32/.firmware/can_signals.h): FL 0|13, FR 13|13, RL 26|13, RR 39|13,
 *  x0.04 km/h; raw >= 8175 (327 km/h) is SNA. Chassis / Party bus.
 *
 *  Drive-unit oil pump 0x395 rear / 0x396 front (Model 3/Y community DBCs):
 *    state bit 0|3, flow target byte1 / flow actual byte2 (x0.06 L/min, 255 SNA)
 *    agree in both DBCs. byte3 / byte4 are temperatures (+offset -40 C) but the
 *    DBCs disagree on which is the fluid. byte5 is a pressure ESTIMATE (x2 kPa)
 *    in joshwardell only; onyx-m2 calls it pump voltage. There is no oil
 *    pressure sensor on these drive units — Tesla estimates pressure from the
 *    pump's electrical data.
 *
 *  BMS_hvBusStatus 0x132: voltage u16 x0.01 V, current s16 x0.1 A (as
 *  fsd_handle_bms_hv). DI_speed 0x257: bit 12|12 x0.08 - 40 km/h, 4095 SNA.
 *  DI_systemStatus 0x118: DI_gear bit 21|3.
 *
 * None of the inverter / oil layouts is confirmed on a refreshed (2021+)
 * Model S/X; IDs can move between firmwares. The screen labels them so.
 */

#include <stdbool.h>
#include <stdint.h>

#define FSD_LIVE_STALE_MS     3000u   // no frame this long -> shown as "--"

#define FSD_LIVE_INV_FRONT    0u
#define FSD_LIVE_INV_REAR     1u
#define FSD_LIVE_INV_REAR_L   2u
#define FSD_LIVE_INV_REAR_R   3u
#define FSD_LIVE_INV_COUNT    4u

#define FSD_LIVE_OIL_FRONT    0u
#define FSD_LIVE_OIL_REAR     1u
#define FSD_LIVE_OIL_COUNT    2u

#define FSD_LIVE_WHEEL_SNA_RAW 8175u  // 327.0 km/h / 0.04
#define FSD_LIVE_U8_SNA        0xFFu

typedef struct {
    bool     current_seen;
    uint32_t current_ms;
    uint16_t motor_current_a; // A
    bool     torque_seen;
    uint32_t torque_ms;
    int16_t  torque_nm;       // Nm (signed)
    int32_t  axle_drpm;       // 0.1 RPM (signed)
} FsdLiveInverter;

typedef struct {
    bool     seen;
    uint32_t seen_ms;
    uint8_t  state;           // 0 STANDBY 1 ENABLE 2 COLD_STARTUP 6 FAULTED 7 SNA
    uint8_t  flow_target_raw; // x0.06 L/min, 255 SNA
    uint8_t  flow_actual_raw;
    uint8_t  temp_a_raw;      // byte3, +offset -40 C, 255 SNA
    uint8_t  temp_b_raw;      // byte4
    uint8_t  pressure_raw;    // byte5, x2 kPa (estimate, one DBC only)
} FsdLiveOilPump;

typedef struct {
    uint32_t        rx_count;
    FsdLiveInverter inv[FSD_LIVE_INV_COUNT];
    FsdLiveOilPump  oil[FSD_LIVE_OIL_COUNT];

    bool     wheels_seen;
    uint32_t wheels_ms;
    uint16_t wheel_raw[4];     // FL FR RL RR, x0.04 km/h

    bool     speed_seen;
    uint32_t speed_ms;
    uint16_t speed_raw;        // DI_speed raw

    bool     gear_seen;
    uint32_t gear_ms;
    uint8_t  gear;

    bool     pack_seen;
    uint32_t pack_ms;
    uint16_t pack_cv;          // 0.01 V
    int16_t  pack_da;          // 0.1 A
} FsdLive;

static inline uint32_t fsd_live_bits(const uint8_t* d, uint8_t start, uint8_t len) {
    uint32_t v = 0;
    for(uint8_t i = 0; i < len; i++) {
        uint8_t n = (uint8_t)(start + i);
        v |= (uint32_t)((d[n >> 3] >> (n & 7)) & 1u) << i;
    }
    return v;
}

static inline bool fsd_live_fresh(bool seen, uint32_t seen_ms, uint32_t now_ms) {
    return seen && (uint32_t)(now_ms - seen_ms) <= FSD_LIVE_STALE_MS;
}

static inline int32_t fsd_live_sbits(const uint8_t* d, uint8_t start, uint8_t len) {
    uint32_t v = fsd_live_bits(d, start, len);
    if(v & (1u << (len - 1))) return (int32_t)v - (int32_t)(1u << len);
    return (int32_t)v;
}

static inline int fsd_live_motor_status_index(uint32_t id) {
    switch(id) {
    case 0x1A5: return FSD_LIVE_INV_FRONT;
    case 0x7EC: case 0x126: return FSD_LIVE_INV_REAR;
    case 0x7EF: case 0x127: return FSD_LIVE_INV_REAR_L;
    case 0x12A: return FSD_LIVE_INV_REAR_R;
    default:    return -1;
    }
}

static inline int fsd_live_torque_index(uint32_t id) {
    switch(id) {
    case 0x186: return FSD_LIVE_INV_FRONT;
    case 0x108: return FSD_LIVE_INV_REAR;
    case 0x188: return FSD_LIVE_INV_REAR_L;
    case 0x10A: return FSD_LIVE_INV_REAR_R;
    default:    return -1;
    }
}

/** Feed one received frame. Returns true when it updated a value. */
static inline bool fsd_live_feed(FsdLive* lv, uint32_t id, const uint8_t* d, uint8_t dlc,
                                 uint32_t now_ms) {
    lv->rx_count++;
    if(dlc > 8) return false;

    int inv = fsd_live_motor_status_index(id);
    if(inv >= 0) {
        if(dlc < 3) return false;
        FsdLiveInverter* x = &lv->inv[inv];
        x->motor_current_a = (uint16_t)fsd_live_bits(d, 11, 11);
        x->current_seen = true;
        x->current_ms = now_ms;
        return true;
    }
    inv = fsd_live_torque_index(id);
    if(inv >= 0) {
        if(dlc < 7) return false;
        FsdLiveInverter* x = &lv->inv[inv];
        x->torque_nm = (int16_t)(fsd_live_sbits(d, 27, 13) * 2);
        x->axle_drpm = fsd_live_sbits(d, 40, 16);
        x->torque_seen = true;
        x->torque_ms = now_ms;
        return true;
    }

    switch(id) {
    case 0x175:
        if(dlc < 7) return false;
        for(uint8_t w = 0; w < 4; w++) lv->wheel_raw[w] = (uint16_t)fsd_live_bits(d, (uint8_t)(w * 13), 13);
        lv->wheels_seen = true;
        lv->wheels_ms = now_ms;
        return true;
    case 0x395:
    case 0x396: {
        if(dlc < 6) return false;
        FsdLiveOilPump* p = &lv->oil[id == 0x396 ? FSD_LIVE_OIL_FRONT : FSD_LIVE_OIL_REAR];
        p->state = d[0] & 0x07u;
        p->flow_target_raw = d[1];
        p->flow_actual_raw = d[2];
        p->temp_a_raw = d[3];
        p->temp_b_raw = d[4];
        p->pressure_raw = d[5];
        p->seen = true;
        p->seen_ms = now_ms;
        return true;
    }
    case 0x132:
        if(dlc < 4) return false;
        lv->pack_cv = (uint16_t)(d[0] | (d[1] << 8));
        lv->pack_da = (int16_t)(uint16_t)(d[2] | (d[3] << 8));
        lv->pack_seen = true;
        lv->pack_ms = now_ms;
        return true;
    case 0x257:
        if(dlc < 3) return false;
        lv->speed_raw = (uint16_t)fsd_live_bits(d, 12, 12);
        lv->speed_seen = true;
        lv->speed_ms = now_ms;
        return true;
    case 0x118:
        if(dlc < 3) return false;
        lv->gear = (uint8_t)fsd_live_bits(d, 21, 3);
        lv->gear_seen = true;
        lv->gear_ms = now_ms;
        return true;
    default:
        return false;
    }
}

// ── formatting helpers (integer only) ───────────────────────────────────────

/** Wheel speed in 0.1 km/h, or -1 when SNA. */
static inline int32_t fsd_live_wheel_dkph(uint16_t raw) {
    if(raw >= FSD_LIVE_WHEEL_SNA_RAW) return -1;
    return (int32_t)raw * 4 / 10;   // raw * 0.04 km/h = raw * 0.4 (0.1 km/h)
}

/** Vehicle speed in 0.1 km/h, or INT32_MIN when SNA. */
static inline int32_t fsd_live_speed_dkph(uint16_t raw) {
    if(raw >= 4095u) return INT32_MIN;
    return (int32_t)raw * 8 / 10 - 400;  // raw*0.08 - 40 km/h, in 0.1 km/h
}

/** Oil flow in 0.01 L/min, or -1 when SNA. */
static inline int32_t fsd_live_oil_flow_clpm(uint8_t raw) {
    return raw == FSD_LIVE_U8_SNA ? -1 : (int32_t)raw * 6;
}

/** Temperature in C from a +40 offset byte, or INT32_MIN when SNA. */
static inline int32_t fsd_live_temp_c(uint8_t raw) {
    return raw == FSD_LIVE_U8_SNA ? INT32_MIN : (int32_t)raw - 40;
}

static inline const char* fsd_live_oil_state_name(uint8_t s) {
    switch(s) {
    case 0:  return "STANDBY";
    case 1:  return "PUMPING";
    case 2:  return "COLD START";
    case 6:  return "FAULTED";
    case 7:  return "SNA";
    default: return "?";
    }
}

static inline const char* fsd_live_gear_name(uint8_t g) {
    switch(g) {
    case 1:  return "P";
    case 2:  return "R";
    case 3:  return "N";
    case 4:  return "D";
    default: return "-";
    }
}

/** Pressure stand-in: actual flow reaches 80 % of target. 1 ok, 0 low, -1 n/a. */
static inline int fsd_live_oil_flow_ok(const FsdLiveOilPump* p) {
    if(p->flow_target_raw == FSD_LIVE_U8_SNA || p->flow_actual_raw == FSD_LIVE_U8_SNA ||
       p->flow_target_raw == 0)
        return -1;
    return (uint32_t)p->flow_actual_raw * 10u >= (uint32_t)p->flow_target_raw * 8u ? 1 : 0;
}
