// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdint.h>

#include "esp_err.h"

#define TCA9554_INPUT_REG       0x00
#define TCA9554_OUTPUT_REG      0x01
#define TCA9554_POLARITY_REG    0x02
#define TCA9554_CONFIG_REG      0x03

esp_err_t tca9554_init_outputs(uint8_t output_mask);
esp_err_t tca9554_read_reg(uint8_t reg, uint8_t *value);
esp_err_t tca9554_write_reg(uint8_t reg, uint8_t value);
esp_err_t tca9554_write_outputs(uint8_t output_mask);
esp_err_t tca9554_read_outputs(uint8_t *output_mask);
