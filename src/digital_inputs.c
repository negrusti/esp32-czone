// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "digital_inputs.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "din";

static digital_inputs_changed_cb_t s_callback;
static void *s_callback_ctx;
static uint8_t s_last_mask;

static uint8_t read_inputs(void)
{
    uint8_t mask = 0;

    for (uint8_t i = 0; i < BOARD_RELAY_COUNT; ++i) {
        const int level = gpio_get_level(BOARD_DIN_GPIOS[i]);
        if (level) {
            mask |= (uint8_t)(1U << i);
        }
    }

    return mask;
}

static void digital_inputs_task(void *arg)
{
    (void)arg;
    s_last_mask = read_inputs();
    ESP_LOGI(TAG, "initial input mask 0x%02x", s_last_mask);

    while (true) {
        const uint8_t mask = read_inputs();
        if (mask != s_last_mask) {
            s_last_mask = mask;
            ESP_LOGI(TAG, "input mask changed to 0x%02x", s_last_mask);
            if (s_callback) {
                s_callback(s_last_mask, s_callback_ctx);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

esp_err_t digital_inputs_init(digital_inputs_changed_cb_t callback, void *ctx)
{
    uint64_t pin_mask = 0;

    for (uint8_t i = 0; i < BOARD_RELAY_COUNT; ++i) {
        pin_mask |= 1ULL << BOARD_DIN_GPIOS[i];
    }

    const gpio_config_t config = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "configure digital inputs");

    s_callback = callback;
    s_callback_ctx = ctx;

    BaseType_t ok = xTaskCreate(digital_inputs_task, "digital_inputs", 4096, NULL, 5, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create digital input task");
    return ESP_OK;
}

uint8_t digital_inputs_get_mask(void)
{
    return s_last_mask;
}
