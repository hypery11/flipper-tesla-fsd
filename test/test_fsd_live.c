// Host tests for fsd_logic/fsd_live.h (Live Data screen decoders).
#include <stdio.h>
#include <string.h>
#include "fsd_live.h"

static int g_pass = 0, g_fail = 0;
#define CHECK(cond) do { if(cond) g_pass++; else { g_fail++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while(0)

// Set an unsigned little-endian field (Tesla bit numbering) in d.
static void put(uint8_t* d, uint8_t start, uint8_t len, uint32_t v) {
    for(uint8_t i = 0; i < len; i++) {
        uint8_t n = (uint8_t)(start + i);
        if((v >> i) & 1u) d[n >> 3] |= (uint8_t)(1u << (n & 7));
        else d[n >> 3] &= (uint8_t)~(1u << (n & 7));
    }
}

static void test_motor_current(void) {
    FsdLive lv; memset(&lv, 0, sizeof(lv));
    uint8_t d[8] = {0};
    put(d, 11, 11, 1234);
    CHECK(fsd_live_feed(&lv, 0x12A, d, 7, 100));           // Plaid rear-right
    CHECK(lv.inv[FSD_LIVE_INV_REAR_R].current_seen);
    CHECK(lv.inv[FSD_LIVE_INV_REAR_R].motor_current_a == 1234);
    CHECK(fsd_live_feed(&lv, 0x7EF, d, 7, 100));           // S/X rear-left
    CHECK(lv.inv[FSD_LIVE_INV_REAR_L].motor_current_a == 1234);
    CHECK(fsd_live_feed(&lv, 0x1A5, d, 7, 100));           // front
    CHECK(lv.inv[FSD_LIVE_INV_FRONT].motor_current_a == 1234);
    CHECK(fsd_live_feed(&lv, 0x7EC, d, 7, 100));           // S/X rear
    CHECK(lv.inv[FSD_LIVE_INV_REAR].motor_current_a == 1234);
    CHECK(!fsd_live_feed(&lv, 0x12A, d, 2, 100));          // too short
}

static void test_torque_and_axle(void) {
    FsdLive lv; memset(&lv, 0, sizeof(lv));
    uint8_t d[8] = {0};
    put(d, 27, 13, (uint32_t)(-250 & 0x1FFF));             // -500 Nm (x2)
    put(d, 40, 16, (uint32_t)(-12345 & 0xFFFF));           // -1234.5 RPM
    CHECK(fsd_live_feed(&lv, 0x188, d, 8, 5));
    CHECK(lv.inv[FSD_LIVE_INV_REAR_L].torque_nm == -500);
    CHECK(lv.inv[FSD_LIVE_INV_REAR_L].axle_drpm == -12345);
    memset(d, 0, sizeof(d));
    put(d, 27, 13, 1000);                                  // +2000 Nm
    put(d, 40, 16, 5000);                                  // 500.0 RPM
    CHECK(fsd_live_feed(&lv, 0x10A, d, 8, 5));
    CHECK(lv.inv[FSD_LIVE_INV_REAR_R].torque_nm == 2000);
    CHECK(lv.inv[FSD_LIVE_INV_REAR_R].axle_drpm == 5000);
}

static void test_wheels(void) {
    FsdLive lv; memset(&lv, 0, sizeof(lv));
    uint8_t d[8] = {0};
    put(d, 0, 13, 2500);    // 100.0 km/h
    put(d, 13, 13, 2525);   // 101.0
    put(d, 26, 13, 0);
    put(d, 39, 13, 8191);   // SNA
    CHECK(fsd_live_feed(&lv, 0x175, d, 8, 1));
    CHECK(fsd_live_wheel_dkph(lv.wheel_raw[0]) == 1000);
    CHECK(fsd_live_wheel_dkph(lv.wheel_raw[1]) == 1010);
    CHECK(fsd_live_wheel_dkph(lv.wheel_raw[2]) == 0);
    CHECK(fsd_live_wheel_dkph(lv.wheel_raw[3]) == -1);
}

static void test_oil(void) {
    FsdLive lv; memset(&lv, 0, sizeof(lv));
    uint8_t d[8] = {0x19, 0x32, 0x3C, 0x5A, 0x6E, 0x96, 0x8C, 0x7D};
    CHECK(fsd_live_feed(&lv, 0x395, d, 8, 1));
    const FsdLiveOilPump* p = &lv.oil[FSD_LIVE_OIL_REAR];
    CHECK(p->state == 1);
    CHECK(strcmp(fsd_live_oil_state_name(p->state), "PUMPING") == 0);
    CHECK(fsd_live_oil_flow_clpm(p->flow_target_raw) == 300);   // 3.00 L/min
    CHECK(fsd_live_oil_flow_clpm(p->flow_actual_raw) == 360);
    CHECK(fsd_live_oil_flow_ok(p) == 1);
    CHECK(fsd_live_temp_c(p->temp_a_raw) == 50);
    CHECK(p->pressure_raw * 2 == 300);
    d[2] = 0x10;  // flow far below target
    CHECK(fsd_live_feed(&lv, 0x395, d, 8, 1));
    CHECK(fsd_live_oil_flow_ok(p) == 0);
    d[1] = 0xFF;
    CHECK(fsd_live_feed(&lv, 0x395, d, 8, 1));
    CHECK(fsd_live_oil_flow_ok(p) == -1);
    CHECK(fsd_live_oil_flow_clpm(0xFF) == -1);
}

static void test_pack_speed_gear(void) {
    FsdLive lv; memset(&lv, 0, sizeof(lv));
    uint8_t b[8] = {0x10, 0x9C, 0x18, 0xFC, 0, 0, 0, 0};    // 400.00 V, -100.0 A
    CHECK(fsd_live_feed(&lv, 0x132, b, 8, 1));
    CHECK(lv.pack_cv == 40000);
    CHECK(lv.pack_da == -1000);
    uint8_t s[8] = {0};
    put(s, 12, 12, 1750);                                    // 100 km/h
    CHECK(fsd_live_feed(&lv, 0x257, s, 8, 1));
    CHECK(fsd_live_speed_dkph(lv.speed_raw) == 1000);
    put(s, 12, 12, 4095);
    CHECK(fsd_live_feed(&lv, 0x257, s, 8, 1));
    CHECK(fsd_live_speed_dkph(lv.speed_raw) == INT32_MIN);
    uint8_t g[8] = {0x39, 0x00, 0x80, 0, 0, 0, 0, 0};        // gear 4 = D
    CHECK(fsd_live_feed(&lv, 0x118, g, 8, 1));
    CHECK(strcmp(fsd_live_gear_name(lv.gear), "D") == 0);
}

static void test_freshness_and_bounds(void) {
    CHECK(fsd_live_fresh(true, 1000, 1000 + FSD_LIVE_STALE_MS));
    CHECK(!fsd_live_fresh(true, 1000, 1001 + FSD_LIVE_STALE_MS));
    CHECK(!fsd_live_fresh(false, 0, 0));
    CHECK(fsd_live_fresh(true, 0xFFFFFF00u, 0x00000010u));   // tick wrap
    FsdLive lv; memset(&lv, 0, sizeof(lv));
    uint8_t d[8] = {0};
    CHECK(!fsd_live_feed(&lv, 0x175, d, 9, 1));               // dlc > 8 rejected
    CHECK(!fsd_live_feed(&lv, 0x123, d, 8, 1));               // unknown id
    CHECK(lv.rx_count == 2);
}

int main(void) {
    test_motor_current();
    test_torque_and_axle();
    test_wheels();
    test_oil();
    test_pack_speed_gear();
    test_freshness_and_bounds();
    printf("test_fsd_live: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
