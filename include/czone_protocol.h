// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "czone_can.h"
#include "esp_err.h"

typedef enum {
    CZONE_CAN_MONITOR_OFF = 0,
    CZONE_CAN_MONITOR_CZONE,
    CZONE_CAN_MONITOR_ZCF,
    CZONE_CAN_MONITOR_ALL,
} czone_can_monitor_mode_t;

void czone_protocol_set_monitor_mode(czone_can_monitor_mode_t mode);
czone_can_monitor_mode_t czone_protocol_get_monitor_mode(void);
const char *czone_protocol_monitor_mode_name(czone_can_monitor_mode_t mode);
esp_err_t czone_protocol_init(void);
esp_err_t czone_protocol_handle_frame(const czone_can_frame_t *frame);
esp_err_t czone_protocol_publish_relay_state(uint8_t relay_mask);
esp_err_t czone_protocol_send_zcf(void);
