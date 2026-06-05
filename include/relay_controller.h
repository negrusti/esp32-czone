// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t relay_controller_init(void);
esp_err_t relay_controller_set_channel(uint8_t channel, bool enabled);
esp_err_t relay_controller_toggle_channel(uint8_t channel);
esp_err_t relay_controller_set_mask(uint8_t mask);
uint8_t relay_controller_get_mask(void);
