// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "i2c_bus.h"

#include "board.h"
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "i2c";
static bool s_installed;

esp_err_t i2c_bus_init(void)
{
    if (s_installed) {
        return ESP_OK;
    }

    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BOARD_I2C_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(BOARD_I2C_PORT, &config), TAG, "configure I2C");
    ESP_RETURN_ON_ERROR(i2c_driver_install(BOARD_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0), TAG, "install I2C driver");
    s_installed = true;
    return ESP_OK;
}

esp_err_t i2c_bus_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        BOARD_I2C_PORT,
        device_addr,
        &reg_addr,
        1,
        data,
        len,
        pdMS_TO_TICKS(100));
}

esp_err_t i2c_bus_write_reg(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data, size_t len)
{
    uint8_t buffer[9] = {reg_addr};

    if (len > sizeof(buffer) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0; i < len; ++i) {
        buffer[i + 1] = data[i];
    }

    return i2c_master_write_to_device(BOARD_I2C_PORT, device_addr, buffer, len + 1, pdMS_TO_TICKS(100));
}
