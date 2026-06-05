// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "relay_controller.h"

#include "board.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "tca9554.h"

static const char *TAG = "relay";

static SemaphoreHandle_t s_lock;
static uint8_t s_relay_mask;

static esp_err_t relay_write_locked(uint8_t mask)
{
    ESP_RETURN_ON_ERROR(tca9554_write_outputs(mask), TAG, "write relay output mask");
    s_relay_mask = mask;
    ESP_LOGI(TAG, "relay mask set to 0x%02x", s_relay_mask);
    return ESP_OK;
}

esp_err_t relay_controller_init(void)
{
    if (!s_lock) {
        s_lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_lock, ESP_ERR_NO_MEM, TAG, "create relay mutex");
    }

    ESP_RETURN_ON_ERROR(tca9554_init_outputs(0x00), TAG, "initialize relay expander");
    s_relay_mask = 0x00;
    ESP_LOGI(TAG, "initialized %d relay outputs", BOARD_RELAY_COUNT);
    return ESP_OK;
}

esp_err_t relay_controller_set_channel(uint8_t channel, bool enabled)
{
    ESP_RETURN_ON_FALSE(channel >= 1 && channel <= BOARD_RELAY_COUNT, ESP_ERR_INVALID_ARG, TAG, "invalid channel %u", channel);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint8_t next = s_relay_mask;
    const uint8_t bit = (uint8_t)(1U << (channel - 1U));

    if (enabled) {
        next |= bit;
    } else {
        next &= (uint8_t)~bit;
    }

    const esp_err_t err = relay_write_locked(next);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t relay_controller_toggle_channel(uint8_t channel)
{
    ESP_RETURN_ON_FALSE(channel >= 1 && channel <= BOARD_RELAY_COUNT, ESP_ERR_INVALID_ARG, TAG, "invalid channel %u", channel);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const uint8_t next = s_relay_mask ^ (uint8_t)(1U << (channel - 1U));
    const esp_err_t err = relay_write_locked(next);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t relay_controller_set_mask(uint8_t mask)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const esp_err_t err = relay_write_locked(mask);
    xSemaphoreGive(s_lock);
    return err;
}

uint8_t relay_controller_get_mask(void)
{
    if (!s_lock) {
        return 0;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    const uint8_t mask = s_relay_mask;
    xSemaphoreGive(s_lock);
    return mask;
}
