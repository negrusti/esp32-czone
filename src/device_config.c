// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "device_config.h"

#include "czone_device_identity.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "device_config";

#define DEVICE_CFG_NVS_NS   "czone"
#define DEVICE_CFG_KEY_DIP  "dipswitch"

static uint8_t s_dipswitch = CZONE_DEVICE_DIPSWITCH;
static bool s_inited;

esp_err_t device_config_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase (%s); reformatting", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_handle_t handle;
    if (nvs_open(DEVICE_CFG_NVS_NS, NVS_READONLY, &handle) == ESP_OK) {
        uint8_t stored = 0;
        if (nvs_get_u8(handle, DEVICE_CFG_KEY_DIP, &stored) == ESP_OK) {
            s_dipswitch = stored;
        }
        nvs_close(handle);
    }

    s_inited = true;
    ESP_LOGI(TAG, "dipswitch = 0x%02x (%u)", s_dipswitch, s_dipswitch);
    return ESP_OK;
}

uint8_t device_config_get_dipswitch(void)
{
    return s_dipswitch;
}

esp_err_t device_config_set_dipswitch(uint8_t dipswitch)
{
    if (!s_inited) {
        const esp_err_t init_err = device_config_init();
        if (init_err != ESP_OK) {
            return init_err;
        }
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open(DEVICE_CFG_NVS_NS, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(handle, DEVICE_CFG_KEY_DIP, dipswitch);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "persist dipswitch failed: %s", esp_err_to_name(err));
        return err;
    }

    s_dipswitch = dipswitch;
    ESP_LOGI(TAG, "dipswitch set to 0x%02x (%u) and saved", dipswitch, dipswitch);
    return ESP_OK;
}
