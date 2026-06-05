// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "board.h"
#include "device_config.h"
#include "czone_device_identity.h"
#include "czone_can.h"
#include "digital_inputs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "relay_controller.h"
#include "usb_terminal.h"
#include "zcf_config.h"

static const char *TAG = "app";

static void log_init_result(const char *name, esp_err_t err)
{
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s initialized", name);
    } else {
        ESP_LOGE(TAG, "%s init failed: %s", name, esp_err_to_name(err));
    }
}

static void on_inputs_changed(uint8_t input_mask, void *ctx)
{
    (void)ctx;
    ESP_LOGI(TAG, "digital input mask changed to 0x%02x", input_mask);
}

void app_main(void)
{
    ESP_LOGI(TAG, "%s booting, preferred N2K address %u", CZONE_DEVICE_NAME, CZONE_DEVICE_PREFERRED_ADDRESS);
    ESP_ERROR_CHECK(usb_terminal_init());
    vTaskDelay(pdMS_TO_TICKS(250));
    board_log_hardware_map();

    /* Load the persisted dipswitch (CZone module address) before anything that
     * publishes PGNs or parses the ZCF uses it. */
    log_init_result("device config", device_config_init());

    log_init_result("I2C", i2c_bus_init());
    log_init_result("relay controller", relay_controller_init());
    log_init_result("CZone CAN", czone_can_init());
    log_init_result("digital inputs", digital_inputs_init(on_inputs_changed, NULL));

    /* Load any previously stored ZCF so circuit->relay switching works across a
     * reboot without re-uploading. Absent/empty config is not an error. */
    if (zcf_config_load() == ESP_OK) {
        ESP_LOGI(TAG, "ZCF config loaded: %u relay mapping(s)", (unsigned)zcf_config_mapping_count());
    } else {
        ESP_LOGI(TAG, "no ZCF relay mapping yet; awaiting config upload");
    }

    ESP_LOGI(TAG, "firmware foundation ready");
}
