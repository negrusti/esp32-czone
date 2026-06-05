// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "tca9554.h"

#include "board.h"
#include "esp_check.h"
#include "i2c_bus.h"

esp_err_t tca9554_read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_bus_read_reg(BOARD_TCA9554_ADDR, reg, value, 1);
}

esp_err_t tca9554_write_reg(uint8_t reg, uint8_t value)
{
    return i2c_bus_write_reg(BOARD_TCA9554_ADDR, reg, &value, 1);
}

esp_err_t tca9554_init_outputs(uint8_t output_mask)
{
    ESP_RETURN_ON_ERROR(tca9554_write_reg(TCA9554_OUTPUT_REG, output_mask), "tca9554", "set initial outputs");
    return tca9554_write_reg(TCA9554_CONFIG_REG, 0x00);
}

esp_err_t tca9554_write_outputs(uint8_t output_mask)
{
    return tca9554_write_reg(TCA9554_OUTPUT_REG, output_mask);
}

esp_err_t tca9554_read_outputs(uint8_t *output_mask)
{
    return tca9554_read_reg(TCA9554_OUTPUT_REG, output_mask);
}
