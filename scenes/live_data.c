/*
 * Live Data scene — read-only dashboard of decoded values: per-motor
 * current and torque (front / rear / Plaid rear-left / rear-right), wheel
 * speeds, axle RPM, drive-unit oil pumps and the HV pack.
 *
 * Always listen-only: the MCP2515 is put in MCP_LISTENONLY and this scene
 * never transmits, whatever Mode is set in Settings. Decoders live in
 * fsd_logic/fsd_live.h (host-tested in test/test_fsd_live.c).
 *
 * The worker only receives and decodes. Every LIVE_REFRESH_MS it copies the
 * values under the mutex and asks the GUI thread to redraw, so the widget is
 * only touched from the GUI thread. [<] / [>] switch pages.
 */

#include "../tesla_fsd_app.h"
#include "../scenes_config/app_scene_functions.h"
#include <stdio.h>
#include <string.h>

#define LIVE_REFRESH_MS  250
#define LIVE_PAGE_COUNT  5

enum {
    LiveEventRefresh = 100,
    LiveEventPrev,
    LiveEventNext,
    LiveEventNoDevice,
};

// 64-bit so the conversion doesn't wrap after ~71 min at a 1 kHz tick.
static uint32_t live_now_ms(void) {
    return (uint32_t)((uint64_t)furi_get_tick() * 1000u / furi_kernel_get_tick_frequency());
}

static const char* const inv_name[FSD_LIVE_INV_COUNT] = {"Front", "Rear", "RearL", "RearR"};

static int32_t live_data_worker(void* context) {
    TeslaFSDApp* app = context;
    MCP2515* mcp = app->mcp_can;
    CANFRAME frame;
    FsdLive lv;
    memset(&lv, 0, sizeof(lv));

    mcp->mode = MCP_LISTENONLY;
    mcp->bitRate = MCP_500KBPS;
    switch(app->mcp_clock) {
    case 1:  mcp->clck = MCP_8MHZ;  break;
    case 2:  mcp->clck = MCP_12MHZ; break;
    default: mcp->clck = MCP_16MHZ; break;
    }
    if(mcp2515_init(mcp) != ERROR_OK) {
        view_dispatcher_send_custom_event(app->view_dispatcher, LiveEventNoDevice);
        return 0;
    }
    // Accept every frame: mask 0 on both buffers.
    init_mask(mcp, 0, 0x000);
    init_mask(mcp, 1, 0x000);
    for(uint8_t i = 0; i < 6; i++) init_filter(mcp, i, 0x000);

    uint32_t last = furi_get_tick();
    while(!(furi_thread_flags_get() & WorkerFlagStop)) {
        bool got = false;
        if(check_receive(mcp) == ERROR_OK && read_can_message(mcp, &frame) == ERROR_OK) {
            uint32_t now_ms = live_now_ms();
            if(!frame.ext && !frame.req)
                fsd_live_feed(&lv, frame.canId, frame.buffer, frame.data_lenght, now_ms);
            got = true;
        }
        uint32_t now = furi_get_tick();
        if((now - last) >= furi_ms_to_ticks(LIVE_REFRESH_MS)) {
            furi_mutex_acquire(app->mutex, FuriWaitForever);
            app->live = lv;
            furi_mutex_release(app->mutex);
            view_dispatcher_send_custom_event(app->view_dispatcher, LiveEventRefresh);
            last = now;
        }
        if(!got) furi_delay_ms(1);
    }
    deinit_mcp2515(mcp);
    return 0;
}

static void live_button_cb(GuiButtonType button, InputType type, void* context) {
    TeslaFSDApp* app = context;
    if(type != InputTypeShort) return;
    view_dispatcher_send_custom_event(
        app->view_dispatcher, button == GuiButtonTypeLeft ? LiveEventPrev : LiveEventNext);
}

// Print "%d.%d" from a value in tenths, sign-safe.
static void tenths(char* out, size_t n, int32_t v) {
    int32_t a = v < 0 ? -v : v;
    snprintf(out, n, "%s%ld.%ld", v < 0 ? "-" : "", (long)(a / 10), (long)(a % 10));
}

static void live_row(TeslaFSDApp* app, uint8_t row, const char* text) {
    widget_add_string_element(app->widget, 0, 13 + row * 10, AlignLeft, AlignTop, FontSecondary, text);
}

static void live_draw(TeslaFSDApp* app) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    FsdLive lv = app->live;
    uint8_t page = app->live_page;
    furi_mutex_release(app->mutex);
    uint32_t now = live_now_ms();

    widget_reset(app->widget);
    static const char* const titles[LIVE_PAGE_COUNT] = {
        "Motors A/Nm", "Wheel speed", "Axle RPM", "Oil pumps", "HV pack"};
    char line[40], a[12], b[12];
    snprintf(line, sizeof(line), "%u/%u %s", (unsigned)(page + 1), (unsigned)LIVE_PAGE_COUNT, titles[page]);
    widget_add_string_element(app->widget, 0, 0, AlignLeft, AlignTop, FontPrimary, line);
    uint8_t row = 0;

    if(lv.rx_count == 0) {
        live_row(app, 0, "No CAN traffic yet.");
        live_row(app, 1, "Listen-only, never TX.");
    } else if(page == 0) {
        // Current (A) and torque (Nm) per motor that is talking.
        for(uint8_t i = 0; i < FSD_LIVE_INV_COUNT && row < 4; i++) {
            const FsdLiveInverter* x = &lv.inv[i];
            bool c = fsd_live_fresh(x->current_seen, x->current_ms, now);
            bool t = fsd_live_fresh(x->torque_seen, x->torque_ms, now);
            if(!c && !t) continue;
            if(c) snprintf(a, sizeof(a), "%uA", (unsigned)x->motor_current_a); else snprintf(a, sizeof(a), "--A");
            if(t) snprintf(b, sizeof(b), "%dNm", (int)x->torque_nm); else snprintf(b, sizeof(b), "--Nm");
            snprintf(line, sizeof(line), "%-5s %6s %8s", inv_name[i], a, b);
            live_row(app, row++, line);
        }
        if(row == 0) live_row(app, 0, "No inverter frames");
    } else if(page == 1) {
        if(fsd_live_fresh(lv.wheels_seen, lv.wheels_ms, now)) {
            static const char* const wn[4] = {"FL", "FR", "RL", "RR"};
            for(uint8_t w = 0; w < 4; w += 2) {
                int32_t v0 = fsd_live_wheel_dkph(lv.wheel_raw[w]);
                int32_t v1 = fsd_live_wheel_dkph(lv.wheel_raw[w + 1]);
                if(v0 < 0) snprintf(a, sizeof(a), "--"); else tenths(a, sizeof(a), v0);
                if(v1 < 0) snprintf(b, sizeof(b), "--"); else tenths(b, sizeof(b), v1);
                snprintf(line, sizeof(line), "%s %6s  %s %6s", wn[w], a, wn[w + 1], b);
                live_row(app, row++, line);
            }
        } else {
            live_row(app, row++, "0x175 not on this bus");
        }
        if(fsd_live_fresh(lv.speed_seen, lv.speed_ms, now)) {
            int32_t s = fsd_live_speed_dkph(lv.speed_raw);
            if(s == INT32_MIN) snprintf(a, sizeof(a), "--"); else tenths(a, sizeof(a), s);
        } else {
            snprintf(a, sizeof(a), "--");
        }
        snprintf(line, sizeof(line), "Car %s km/h  Gear %s", a,
                 fsd_live_fresh(lv.gear_seen, lv.gear_ms, now) ? fsd_live_gear_name(lv.gear) : "-");
        live_row(app, row++, line);
    } else if(page == 2) {
        for(uint8_t i = 0; i < FSD_LIVE_INV_COUNT && row < 4; i++) {
            const FsdLiveInverter* x = &lv.inv[i];
            if(!fsd_live_fresh(x->torque_seen, x->torque_ms, now)) continue;
            tenths(a, sizeof(a), x->axle_drpm);
            snprintf(line, sizeof(line), "%-5s %9s rpm", inv_name[i], a);
            live_row(app, row++, line);
        }
        if(row == 0) live_row(app, 0, "No torque frames");
    } else if(page == 3) {
        static const char* const on[FSD_LIVE_OIL_COUNT] = {"F", "R"};
        for(uint8_t i = 0; i < FSD_LIVE_OIL_COUNT; i++) {
            const FsdLiveOilPump* p = &lv.oil[i];
            if(!fsd_live_fresh(p->seen, p->seen_ms, now)) {
                snprintf(line, sizeof(line), "%s: not seen", on[i]);
                live_row(app, row++, line);
                continue;
            }
            int ok = fsd_live_oil_flow_ok(p);
            int32_t fl = fsd_live_oil_flow_clpm(p->flow_actual_raw);
            if(fl < 0) snprintf(a, sizeof(a), "--");
            else snprintf(a, sizeof(a), "%ld.%02ld", (long)(fl / 100), (long)(fl % 100));
            snprintf(line, sizeof(line), "%s %-9s %sL/m %s", on[i], fsd_live_oil_state_name(p->state), a,
                     ok == 1 ? "OK" : ok == 0 ? "LOW" : "");
            live_row(app, row++, line);
            int32_t tb = fsd_live_temp_c(p->temp_b_raw);
            if(tb == INT32_MIN) snprintf(a, sizeof(a), "--"); else snprintf(a, sizeof(a), "%ld", (long)tb);
            if(p->pressure_raw == FSD_LIVE_U8_SNA) snprintf(b, sizeof(b), "--");
            else snprintf(b, sizeof(b), "%u", (unsigned)(p->pressure_raw * 2u));
            snprintf(line, sizeof(line), "  %sC  P~%skPa?", a, b);
            live_row(app, row++, line);
        }
    } else {
        if(fsd_live_fresh(lv.pack_seen, lv.pack_ms, now)) {
            snprintf(line, sizeof(line), "%u.%02u V", (unsigned)(lv.pack_cv / 100), (unsigned)(lv.pack_cv % 100));
            live_row(app, row++, line);
            tenths(a, sizeof(a), lv.pack_da);
            snprintf(line, sizeof(line), "%s A", a);
            live_row(app, row++, line);
            // kW = V * A: (cV/100) * (dA/10) / 1000  ->  cV * dA / 1e6, shown in 0.1 kW
            int32_t dkw = (int32_t)((int64_t)lv.pack_cv * lv.pack_da / 100000);
            tenths(a, sizeof(a), dkw);
            snprintf(line, sizeof(line), "%s kW", a);
            live_row(app, row++, line);
        } else {
            live_row(app, row++, "0x132 not seen");
        }
        snprintf(line, sizeof(line), "Frames: %lu", (unsigned long)lv.rx_count);
        live_row(app, row++, line);
    }

    widget_add_button_element(app->widget, GuiButtonTypeLeft, "", live_button_cb, app);
    widget_add_button_element(app->widget, GuiButtonTypeRight, "", live_button_cb, app);
}

void tesla_fsd_scene_live_data_on_enter(void* context) {
    TeslaFSDApp* app = context;
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    memset(&app->live, 0, sizeof(app->live));
    furi_mutex_release(app->mutex);

    live_draw(app);
    view_dispatcher_switch_to_view(app->view_dispatcher, TeslaFSDViewWidget);

    app->worker_thread = furi_thread_alloc_ex("TeslaLive", 4096, live_data_worker, app);
    furi_thread_start(app->worker_thread);
}

bool tesla_fsd_scene_live_data_on_event(void* context, SceneManagerEvent event) {
    TeslaFSDApp* app = context;
    if(event.type != SceneManagerEventTypeCustom) return false;
    switch(event.event) {
    case LiveEventPrev:
        app->live_page = (uint8_t)((app->live_page + LIVE_PAGE_COUNT - 1) % LIVE_PAGE_COUNT);
        live_draw(app);
        return true;
    case LiveEventNext:
        app->live_page = (uint8_t)((app->live_page + 1) % LIVE_PAGE_COUNT);
        live_draw(app);
        return true;
    case LiveEventRefresh:
        live_draw(app);
        return true;
    case LiveEventNoDevice:
        widget_reset(app->widget);
        widget_add_string_multiline_element(
            app->widget, 64, 28, AlignCenter, AlignCenter, FontPrimary, "CAN Module\nNot Found");
        return true;
    default:
        return false;
    }
}

void tesla_fsd_scene_live_data_on_exit(void* context) {
    TeslaFSDApp* app = context;
    if(app->worker_thread) {
        furi_thread_flags_set(furi_thread_get_id(app->worker_thread), WorkerFlagStop);
        furi_thread_join(app->worker_thread);
        furi_thread_free(app->worker_thread);
        app->worker_thread = NULL;
    }
    widget_reset(app->widget);
}
