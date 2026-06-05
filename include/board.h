// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include "driver/gpio.h"

#define BOARD_I2C_PORT          I2C_NUM_0
#define BOARD_I2C_SDA_GPIO      GPIO_NUM_42
#define BOARD_I2C_SCL_GPIO      GPIO_NUM_41
#define BOARD_I2C_FREQ_HZ       100000

#define BOARD_TCA9554_ADDR      0x20
#define BOARD_RELAY_COUNT       8

#define BOARD_CAN_TX_GPIO       GPIO_NUM_17
#define BOARD_CAN_RX_GPIO       GPIO_NUM_18

#define BOARD_RGB_GPIO          GPIO_NUM_38
#define BOARD_BUZZER_GPIO       GPIO_NUM_46

extern const gpio_num_t BOARD_DIN_GPIOS[BOARD_RELAY_COUNT];

void board_log_hardware_map(void);
