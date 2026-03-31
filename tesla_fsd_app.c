#include "tesla_fsd_app.h"
#include "scenes_config/app_scene_functions.h"

static bool tesla_fsd_custom_event_callback(void* context, uint32_t event) {
    TeslaFSDApp* app = context;
    return scene_manager_handle_custom_event(app->scene_manager, event);
}

static bool tesla_fsd_back_event_callback(void* context) {
    TeslaFSDApp* app = context;
    return scene_manager_handle_back_event(app->scene_manager);
}

TeslaFSDApp* tesla_fsd_app_alloc(void) {
    TeslaFSDApp* app = malloc(sizeof(TeslaFSDApp));
    memset(app, 0, sizeof(TeslaFSDApp));

    // MCP2515
    app->mcp_can = malloc(sizeof(MCP2515));
    memset(app->mcp_can, 0, sizeof(MCP2515));

    // Mutex
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);

    // GUI
    app->gui = furi_record_open(RECORD_GUI);

    // Scene manager
    app->scene_manager = scene_manager_alloc(&tesla_fsd_scene_handlers, app);

    // View dispatcher
    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_custom_event_callback(app->view_dispatcher, tesla_fsd_custom_event_callback);
    view_dispatcher_set_navigation_event_callback(app->view_dispatcher, tesla_fsd_back_event_callback);
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);

    // Submenu view
    app->submenu = submenu_alloc();
    view_dispatcher_add_view(app->view_dispatcher, TeslaFSDViewSubmenu, submenu_get_view(app->submenu));

    // Widget view
    app->widget = widget_alloc();
    view_dispatcher_add_view(app->view_dispatcher, TeslaFSDViewWidget, widget_get_view(app->widget));

    // Default state
    app->hw_version = TeslaHW_Unknown;
    fsd_state_init(&app->fsd_state, TeslaHW_Unknown);

    return app;
}

void tesla_fsd_app_free(TeslaFSDApp* app) {
    view_dispatcher_remove_view(app->view_dispatcher, TeslaFSDViewSubmenu);
    view_dispatcher_remove_view(app->view_dispatcher, TeslaFSDViewWidget);

    submenu_free(app->submenu);
    widget_free(app->widget);

    scene_manager_free(app->scene_manager);
    view_dispatcher_free(app->view_dispatcher);

    furi_record_close(RECORD_GUI);

    furi_mutex_free(app->mutex);

    free(app->mcp_can);
    free(app);
}

int32_t tesla_fsd_main(void* p) {
    UNUSED(p);
    TeslaFSDApp* app = tesla_fsd_app_alloc();

    scene_manager_next_scene(app->scene_manager, tesla_fsd_scene_main_menu);
    view_dispatcher_run(app->view_dispatcher);

    tesla_fsd_app_free(app);
    return 0;
}
