// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "czone_can.h"

#include <string.h>

#include "board.h"
#include "czone_protocol.h"
#include "driver/twai.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "czone_can";

#define CAN_APP_RX_QUEUE_LEN 512

static QueueHandle_t s_app_rx_queue;
static czone_can_stats_t s_stats;

static void czone_rx_task(void *arg)
{
    (void)arg;

    while (true) {
        uint32_t alerts = 0;
        esp_err_t err = twai_read_alerts(&alerts, pdMS_TO_TICKS(1000));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "read alerts failed: %s", esp_err_to_name(err));
            continue;
        }

        if (alerts & TWAI_ALERT_BUS_OFF) {
            ++s_stats.twai_bus_off;
            ESP_LOGE(TAG, "CAN bus off, initiating recovery");
            ESP_ERROR_CHECK_WITHOUT_ABORT(twai_initiate_recovery());
            continue;
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            ++s_stats.twai_bus_recovered;
            ESP_LOGW(TAG, "CAN bus recovered, restarting TWAI");
            ESP_ERROR_CHECK_WITHOUT_ABORT(twai_start());
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            ++s_stats.twai_rx_queue_full;
            ESP_LOGW(TAG, "CAN RX queue full (%lu)", s_stats.twai_rx_queue_full);
        }
        if (alerts & TWAI_ALERT_RX_FIFO_OVERRUN) {
            ++s_stats.twai_rx_fifo_overrun;
        }
        if (alerts & TWAI_ALERT_BUS_ERROR) {
            ++s_stats.twai_bus_error;
        }
        if (alerts & TWAI_ALERT_ARB_LOST) {
            ++s_stats.twai_arb_lost;
        }
        if (alerts & TWAI_ALERT_ERR_PASS) {
            ++s_stats.twai_err_passive;
        }
        if (alerts & TWAI_ALERT_TX_FAILED) {
            ++s_stats.twai_tx_failed;
        }
        if (alerts & TWAI_ALERT_TX_RETRIED) {
            ++s_stats.twai_tx_retried;
        }
        if (alerts & TWAI_ALERT_PERIPH_RESET) {
            ++s_stats.twai_periph_reset;
        }

        if (alerts & TWAI_ALERT_RX_DATA) {
            twai_message_t message = {0};
            while (twai_receive(&message, 0) == ESP_OK) {
                czone_can_frame_t frame = {
                    .can_id = message.identifier,
                    .extended = message.extd,
                    .dlc = message.data_length_code,
                };
                memcpy(frame.data, message.data, frame.dlc <= 8 ? frame.dlc : 8);
                if (xQueueSend(s_app_rx_queue, &frame, 0) == pdPASS) {
                    ++s_stats.app_rx_enqueued;
                } else {
                    ++s_stats.app_rx_queue_full;
                }
            }
        }
    }
}

static void czone_protocol_rx_task(void *arg)
{
    (void)arg;
    czone_can_frame_t frame;

    while (true) {
        if (xQueueReceive(s_app_rx_queue, &frame, portMAX_DELAY) == pdPASS) {
            ++s_stats.app_rx_processed;
            ESP_ERROR_CHECK_WITHOUT_ABORT(czone_protocol_handle_frame(&frame));
        }
    }
}

esp_err_t czone_can_init(void)
{
    s_app_rx_queue = xQueueCreate(CAN_APP_RX_QUEUE_LEN, sizeof(czone_can_frame_t));
    ESP_RETURN_ON_FALSE(s_app_rx_queue != NULL, ESP_ERR_NO_MEM, TAG, "create CAN app RX queue");

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(BOARD_CAN_TX_GPIO, BOARD_CAN_RX_GPIO, TWAI_MODE_NORMAL);
    general.rx_queue_len = 128;
    general.tx_queue_len = 16;

    twai_timing_config_t timing = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_RETURN_ON_ERROR(twai_driver_install(&general, &timing, &filter), TAG, "install TWAI driver");
    ESP_RETURN_ON_ERROR(twai_start(), TAG, "start TWAI driver");

    const uint32_t alerts =
        TWAI_ALERT_RX_DATA |
        TWAI_ALERT_RX_QUEUE_FULL |
        TWAI_ALERT_RX_FIFO_OVERRUN |
        TWAI_ALERT_BUS_ERROR |
        TWAI_ALERT_ARB_LOST |
        TWAI_ALERT_TX_FAILED |
        TWAI_ALERT_TX_RETRIED |
        TWAI_ALERT_PERIPH_RESET |
        TWAI_ALERT_BUS_OFF |
        TWAI_ALERT_BUS_RECOVERED |
        TWAI_ALERT_ERR_PASS;
    ESP_RETURN_ON_ERROR(twai_reconfigure_alerts(alerts, NULL), TAG, "configure TWAI alerts");
    ESP_RETURN_ON_ERROR(czone_protocol_init(), TAG, "start CZone protocol");

    BaseType_t ok = xTaskCreate(czone_rx_task, "czone_can_rx", 4096, NULL, 6, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create CAN RX task");
    ok = xTaskCreate(czone_protocol_rx_task, "czone_proto_rx", 6144, NULL, 5, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create CAN protocol RX task");

    ESP_LOGI(TAG, "TWAI started at %d kbit/s", CONFIG_CZONE_CAN_BITRATE_KBPS);
    return ESP_OK;
}

void czone_can_get_stats(czone_can_stats_t *stats)
{
    if (stats) {
        *stats = s_stats;
        twai_status_info_t status = {0};
        if (twai_get_status_info(&status) == ESP_OK) {
            stats->hw_msgs_to_rx = status.msgs_to_rx;
            stats->hw_tx_error_counter = status.tx_error_counter;
            stats->hw_rx_error_counter = status.rx_error_counter;
            stats->hw_tx_failed_count = status.tx_failed_count;
            stats->hw_rx_missed_count = status.rx_missed_count;
            stats->hw_rx_overrun_count = status.rx_overrun_count;
            stats->hw_arb_lost_count = status.arb_lost_count;
            stats->hw_bus_error_count = status.bus_error_count;
        }
    }
}

esp_err_t czone_can_send(const czone_can_frame_t *frame)
{
    ESP_RETURN_ON_FALSE(frame && frame->dlc <= 8, ESP_ERR_INVALID_ARG, TAG, "invalid CAN frame");

    twai_message_t message = {
        .identifier = frame->can_id,
        .data_length_code = frame->dlc,
        .extd = frame->extended,
        .rtr = 0,
    };
    memcpy(message.data, frame->data, frame->dlc);

    return twai_transmit(&message, pdMS_TO_TICKS(100));
}

esp_err_t czone_can_publish_relay_state(uint8_t relay_mask)
{
    return czone_protocol_publish_relay_state(relay_mask);
}
