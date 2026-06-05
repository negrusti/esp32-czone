// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "board.h"

#include "esp_log.h"

static const char *TAG = "board";

const gpio_num_t BOARD_DIN_GPIOS[BOARD_RELAY_COUNT] = {
    GPIO_NUM_4,
    GPIO_NUM_5,
    GPIO_NUM_6,
    GPIO_NUM_7,
    GPIO_NUM_8,
    GPIO_NUM_9,
    GPIO_NUM_10,
    GPIO_NUM_11,
};

void board_log_hardware_map(void)
{
    ESP_LOGI(TAG, "Waveshare ESP32-S3-POE-ETH-8DI-8RO hardware map");
    ESP_LOGI(TAG, "I2C: SDA=%d SCL=%d addr=0x%02x", BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO, BOARD_TCA9554_ADDR);
    ESP_LOGI(TAG, "TWAI/CAN: TX=%d RX=%d bitrate=%d kbit/s", BOARD_CAN_TX_GPIO, BOARD_CAN_RX_GPIO, CONFIG_CZONE_CAN_BITRATE_KBPS);
    ESP_LOGI(TAG, "DIN: GPIO 4,5,6,7,8,9,10,11");
}
