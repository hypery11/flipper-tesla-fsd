#include <cassert>
#include <cstdio>
#include <cstring>
#include "tlssc_norestart.h"

static TlsscNrCache valid_cache(uint8_t bus = 0) {
    TlsscNrCache cache = {};
    cache.bus = bus;
    cache.len = TLSSC_NR_FRAME_LEN;
    cache.data[0] = 0xC5;
    cache.data[1] = 0x12;
    cache.data[7] = 0xA6;
    cache.data[0] = (uint8_t)((cache.data[0] & 0xC0u) | 0x1Bu);
    return cache;
}

static void test_cache_validation_and_load() {
    TlsscNrCache cache = valid_cache();
    assert(tlssc_nr_cache_valid(cache));
    cache.data[0] &= 0xC0u;
    assert(!tlssc_nr_cache_valid(cache));
    cache = valid_cache();
    cache.len = 7;
    assert(!tlssc_nr_cache_valid(cache));
    cache = valid_cache();
    cache.bus = 2;
    assert(!tlssc_nr_cache_valid(cache));

    TlsscNrRuntime runtime = {};
    tlssc_nr_load(&runtime, cache);
    assert(!runtime.cache_valid && !runtime.save_attempted);

    cache = valid_cache(1);
    tlssc_nr_load(&runtime, cache);
    assert(runtime.cache_valid && runtime.save_attempted);
    assert(runtime.cache.bus == 1);
    assert(memcmp(runtime.cache.data, cache.data, TLSSC_NR_FRAME_LEN) == 0);
}

static void test_observe_and_patch() {
    TlsscNrRuntime runtime = {};
    uint8_t frame[TLSSC_NR_FRAME_LEN] = {0xC5, 1, 2, 3, 4, 5, 6, 7};
    assert(tlssc_nr_observe(&runtime, frame, sizeof(frame), 1, 5000));
    assert(runtime.cache_valid && runtime.cache.bus == 1);
    assert(runtime.cache.data[0] == 0xDB);
    assert(runtime.cache.data[1] == 1 && runtime.cache.data[7] == 7);
    assert(runtime.stock_rx_ms == 5000);
    assert(!tlssc_nr_observe(&runtime, frame, sizeof(frame), 1, 6000));
    frame[7] = 8;
    assert(tlssc_nr_observe(&runtime, frame, sizeof(frame), 1, 7000));
    assert(!tlssc_nr_observe(&runtime, frame, 7, 1, 8000));
    assert(!tlssc_nr_observe(&runtime, frame, sizeof(frame), 2, 9000));
}

static void test_due_gates_and_pacing() {
    TlsscNrRuntime runtime = {};
    tlssc_nr_load(&runtime, valid_cache(1));
    assert(!tlssc_nr_due(&runtime, 0, true, true, 1000));
    assert(!tlssc_nr_due(&runtime, 1, false, true, 1000));
    assert(!tlssc_nr_due(&runtime, 1, true, false, 1000));
    assert(runtime.armed_ms == 0);
    assert(tlssc_nr_due(&runtime, 1, true, true, 1000));
    assert(runtime.armed_ms == 1000);
    tlssc_nr_record_result(&runtime, true, 1000);
    assert(runtime.burst == 1 && runtime.replay_ok == 1 && runtime.replay_fail == 0);
    assert(!tlssc_nr_due(&runtime, 1, true, true, 1099));
    assert(tlssc_nr_due(&runtime, 1, true, true, 1100));
    tlssc_nr_record_result(&runtime, false, 1100);
    assert(runtime.burst == 1 && runtime.replay_ok == 1 && runtime.replay_fail == 1);
    assert(tlssc_nr_due(&runtime, 1, true, true, 1200));
}

static void test_stock_frame_yield() {
    TlsscNrRuntime runtime = {};
    uint8_t frame[TLSSC_NR_FRAME_LEN] = {0x05, 0, 0, 0, 0, 0, 0, 0};
    assert(tlssc_nr_observe(&runtime, frame, sizeof(frame), 0, 5000));
    assert(!tlssc_nr_due(&runtime, 0, true, true, 5000));
    assert(!tlssc_nr_due(&runtime, 0, true, true, 6999));
    assert(tlssc_nr_due(&runtime, 0, true, true, 7000));
    assert(tlssc_nr_observe(&runtime, frame, sizeof(frame), 0, 7100) == false);
    assert(!tlssc_nr_due(&runtime, 0, true, true, 9099));
    assert(tlssc_nr_due(&runtime, 0, true, true, 9100));
}

static void test_window_and_frame_cap() {
    TlsscNrRuntime runtime = {};
    tlssc_nr_load(&runtime, valid_cache());
    assert(tlssc_nr_due(&runtime, 0, true, true, 1000));
    assert(!tlssc_nr_due(&runtime, 0, true, true, 61001));

    runtime = TlsscNrRuntime{};
    tlssc_nr_load(&runtime, valid_cache());
    for (uint16_t i = 0; i < TLSSC_NR_MAX_FRAMES; i++) {
        uint32_t now = 100u + (uint32_t)i * TLSSC_NR_PERIOD_MS;
        assert(tlssc_nr_due(&runtime, 0, true, true, now));
        tlssc_nr_record_result(&runtime, true, now);
    }
    assert(runtime.burst == TLSSC_NR_MAX_FRAMES);
    assert(!tlssc_nr_due(&runtime, 0, true, true, 60000));
}

static void test_rearm_and_wrap() {
    TlsscNrRuntime runtime = {};
    tlssc_nr_load(&runtime, valid_cache());
    assert(tlssc_nr_due(&runtime, 0, true, true, 1000));
    tlssc_nr_record_result(&runtime, true, 1000);
    tlssc_nr_rearm(&runtime, 2000);
    assert(runtime.armed_ms == 0 && runtime.last_replay_ms == 0 && runtime.burst == 0);
    assert(tlssc_nr_due(&runtime, 0, true, true, 2000));

    runtime = TlsscNrRuntime{};
    tlssc_nr_load(&runtime, valid_cache());
    assert(tlssc_nr_due(&runtime, 0, true, true, 0xFFFFFF00u));
    tlssc_nr_record_result(&runtime, true, 0xFFFFFF00u);
    assert(!tlssc_nr_due(&runtime, 0, true, true, 0xFFFFFF32u));
    assert(tlssc_nr_due(&runtime, 0, true, true, 0xFFFFFF64u));
}

static void test_save_retry_and_bus_change() {
    TlsscNrRuntime runtime = {};
    uint8_t frame[TLSSC_NR_FRAME_LEN] = {0x05, 1, 2, 3, 4, 5, 6, 7};
    assert(tlssc_nr_observe(&runtime, frame, sizeof(frame), 0, 100));
    assert(runtime.cache_valid && !runtime.save_attempted);
    assert(tlssc_nr_should_save(&runtime, 100));
    tlssc_nr_note_save_attempt(&runtime, 100);
    assert(!tlssc_nr_should_save(&runtime, 5099));
    assert(tlssc_nr_should_save(&runtime, 5100));
    tlssc_nr_mark_saved(&runtime);
    assert(runtime.save_attempted && !tlssc_nr_should_save(&runtime, 6000));

    tlssc_nr_load(&runtime, valid_cache(1));
    assert(runtime.save_attempted);
    assert(tlssc_nr_observe(&runtime, frame, sizeof(frame), 0, 7000));
    assert(!runtime.save_attempted && runtime.cache.bus == 0);
    assert(tlssc_nr_should_save(&runtime, 7000));
}

static void test_vehicle_wake_edge() {
    TlsscNrRuntime runtime = {};
    assert(!tlssc_nr_wake_edge(&runtime, 0, 100));
    assert(runtime.bus_quiet);
    assert(tlssc_nr_wake_edge(&runtime, 100, 200));
    assert(!runtime.bus_quiet);
    assert(!tlssc_nr_wake_edge(&runtime, 200, 300));
    assert(!tlssc_nr_wake_edge(&runtime, 200, 30200));
    assert(runtime.bus_quiet);
    assert(tlssc_nr_wake_edge(&runtime, 30200, 30201));

    runtime = TlsscNrRuntime{};
    assert(!tlssc_nr_wake_edge(&runtime, 0xFFFFFF00u, 0xFFFFFF10u));
    assert(!tlssc_nr_wake_edge(&runtime, 0xFFFFFF00u, 0x00007530u));
    assert(runtime.bus_quiet);
    assert(tlssc_nr_wake_edge(&runtime, 0x00007530u, 0x00007531u));
}

int main() {
    test_cache_validation_and_load();
    test_observe_and_patch();
    test_due_gates_and_pacing();
    test_stock_frame_yield();
    test_window_and_frame_cap();
    test_rearm_and_wrap();
    test_save_retry_and_bus_change();
    test_vehicle_wake_edge();
    puts("TLSSC no-restart tests passed: cache, patch, gates, pacing, yield, window, cap, wrap, save retry, wake");
}
