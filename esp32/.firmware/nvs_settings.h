#pragma once

#include "fsd_handler.h"

/** Load persisted toggle settings from NVS into state.
 *  Missing keys use the current state values as defaults. */
void nvs_settings_load(FSDState *state);

/** Save all toggle settings from state to NVS. */
void nvs_settings_save(const FSDState *state);
