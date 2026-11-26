#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TLSSC_NR_FRAME_LEN 8u
#define TLSSC_NR_WINDOW_MS 60000u
#define TLSSC_NR_PERIOD_MS 100u
#define TLSSC_NR_MAX_FRAMES 600u
#define TLSSC_NR_STOCK_FRESH_MS 2000u
#define TLSSC_NR_SAVE_RETRY_MS 5000u
#define TLSSC_NR_WAKE_QUIET_MS 30000u

struct TlsscNrCache {
    uint8_t bus;
    uint8_t len;
    uint8_t data[TLSSC_NR_FRAME_LEN];
};

struct TlsscNrRuntime {
    TlsscNrCache cache;
    bool cache_valid;
    bool save_attempted;
    uint32_t save_attempt_ms;
    bool bus_quiet;
    uint32_t stock_rx_ms;
    uint32_t armed_ms;
    uint32_t last_replay_ms;
    uint16_t burst;
    uint32_t replay_ok;
    uint32_t replay_fail;
    uint32_t replay_busy;
};

static inline bool tlssc_nr_cache_valid(const TlsscNrCache &cache) {
    return cache.bus < 2u && cache.len >= 1u && cache.len <= TLSSC_NR_FRAME_LEN &&
           (cache.data[0] & 0x3Fu) == 0x1Bu;
}

static inline void tlssc_nr_load(TlsscNrRuntime *runtime, const TlsscNrCache &cache) {
    *runtime = TlsscNrRuntime{};
    if (!tlssc_nr_cache_valid(cache)) return;
    runtime->cache = cache;
    runtime->cache_valid = true;
    runtime->save_attempted = true;
}

static inline bool tlssc_nr_should_save(const TlsscNrRuntime *runtime,
                                        uint32_t now_ms) {
    return runtime->cache_valid && !runtime->save_attempted &&
           (runtime->save_attempt_ms == 0 ||
            (uint32_t)(now_ms - runtime->save_attempt_ms) >= TLSSC_NR_SAVE_RETRY_MS);
}

static inline void tlssc_nr_note_save_attempt(TlsscNrRuntime *runtime,
                                              uint32_t now_ms) {
    runtime->save_attempt_ms = now_ms ? now_ms : 1u;
}

static inline void tlssc_nr_mark_saved(TlsscNrRuntime *runtime) {
    runtime->save_attempted = true;
}

static inline bool tlssc_nr_wake_edge(TlsscNrRuntime *runtime,
                                      uint32_t last_rx_ms, uint32_t now_ms) {
    bool quiet = last_rx_ms == 0 ||
                 (uint32_t)(now_ms - last_rx_ms) >= TLSSC_NR_WAKE_QUIET_MS;
    bool woke = runtime->bus_quiet && !quiet;
    runtime->bus_quiet = quiet;
    return woke;
}

static inline bool tlssc_nr_observe(TlsscNrRuntime *runtime, const uint8_t *data,
                                   uint8_t len, uint8_t bus, uint32_t now_ms) {
    if (!data || len < 1u || len > TLSSC_NR_FRAME_LEN || bus >= 2u) return false;
    runtime->stock_rx_ms = now_ms;

    uint8_t patched[TLSSC_NR_FRAME_LEN] = {};
    memcpy(patched, data, len);
    patched[0] = (uint8_t)((patched[0] & 0xC0u) | 0x1Bu);

    bool changed = !runtime->cache_valid || runtime->cache.bus != bus ||
                   runtime->cache.len != len ||
                   memcmp(runtime->cache.data, patched, len) != 0;
    if (!changed) return false;

    runtime->cache.bus = bus;
    runtime->cache.len = len;
    memcpy(runtime->cache.data, patched, len);
    runtime->cache_valid = true;
    return true;
}

static inline void tlssc_nr_rearm(TlsscNrRuntime *runtime, uint32_t now_ms) {
    (void)now_ms;
    runtime->armed_ms = 0;
    runtime->last_replay_ms = 0;
    runtime->burst = 0;
}

static inline bool tlssc_nr_due(TlsscNrRuntime *runtime, uint8_t bus,
                                bool bus_ready, bool tx_allowed, uint32_t now_ms) {
    if (!runtime->cache_valid || bus != runtime->cache.bus ||
        !bus_ready || !tx_allowed) return false;
    if (runtime->armed_ms == 0) runtime->armed_ms = now_ms ? now_ms : 1u;
    if ((uint32_t)(now_ms - runtime->armed_ms) > TLSSC_NR_WINDOW_MS) return false;
    if (runtime->burst >= TLSSC_NR_MAX_FRAMES) return false;
    if (runtime->stock_rx_ms &&
        (uint32_t)(now_ms - runtime->stock_rx_ms) < TLSSC_NR_STOCK_FRESH_MS)
        return false;
    if (runtime->last_replay_ms &&
        (uint32_t)(now_ms - runtime->last_replay_ms) < TLSSC_NR_PERIOD_MS)
        return false;
    return true;
}

static inline void tlssc_nr_record_result(TlsscNrRuntime *runtime, bool sent,
                                         bool busy, uint32_t now_ms) {
    if (busy) {
        runtime->replay_busy++;
        return;
    }
    runtime->last_replay_ms = now_ms ? now_ms : 1u;
    runtime->burst++;
    if (sent) runtime->replay_ok++;
    else runtime->replay_fail++;
}

static inline void tlssc_nr_copy_payload(const TlsscNrRuntime *runtime,
                                         uint8_t *out) {
    memcpy(out, runtime->cache.data, runtime->cache.len);
}
