// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdint.h>

#include "esp_err.h"

typedef void (*digital_inputs_changed_cb_t)(uint8_t input_mask, void *ctx);

esp_err_t digital_inputs_init(digital_inputs_changed_cb_t callback, void *ctx);
uint8_t digital_inputs_get_mask(void);
