// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t i2c_bus_init(void);
esp_err_t i2c_bus_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_bus_write_reg(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, size_t len);
