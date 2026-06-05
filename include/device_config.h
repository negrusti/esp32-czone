// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdint.h>

#include "esp_err.h"

/*
 * Runtime, NVS-backed device settings.
 *
 * The CZone "dipswitch" is this module's network address. It was historically a
 * compile-time constant (CZONE_DEVICE_DIPSWITCH); it is now overridable at run
 * time and persisted in NVS so it survives power cuts. The compile-time value is
 * used as the default the first time the device boots with empty NVS.
 */

/* Initialise NVS and load the persisted dipswitch (default
 * CZONE_DEVICE_DIPSWITCH). Call once, early in app startup. Safe to call
 * repeatedly. */
esp_err_t device_config_init(void);

/* Current dipswitch / CZone module address. Cheap (cached). */
uint8_t device_config_get_dipswitch(void);

/* Persist a new dipswitch to NVS and update the cached value. */
esp_err_t device_config_set_dipswitch(uint8_t dipswitch);
