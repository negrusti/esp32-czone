// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint32_t can_id;
    bool extended;
    uint8_t dlc;
    uint8_t data[8];
} czone_can_frame_t;

typedef struct {
    uint32_t twai_rx_queue_full;
    uint32_t twai_rx_fifo_overrun;
    uint32_t twai_bus_error;
    uint32_t twai_arb_lost;
    uint32_t twai_err_passive;
    uint32_t twai_bus_off;
    uint32_t twai_bus_recovered;
    uint32_t twai_tx_failed;
    uint32_t twai_tx_retried;
    uint32_t twai_periph_reset;
    uint32_t app_rx_queue_full;
    uint32_t app_rx_enqueued;
    uint32_t app_rx_processed;
    uint32_t hw_msgs_to_rx;
    uint32_t hw_tx_error_counter;
    uint32_t hw_rx_error_counter;
    uint32_t hw_tx_failed_count;
    uint32_t hw_rx_missed_count;
    uint32_t hw_rx_overrun_count;
    uint32_t hw_arb_lost_count;
    uint32_t hw_bus_error_count;
} czone_can_stats_t;

esp_err_t czone_can_init(void);
void czone_can_get_stats(czone_can_stats_t *stats);
esp_err_t czone_can_send(const czone_can_frame_t *frame);
esp_err_t czone_can_publish_relay_state(uint8_t relay_mask);
