// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include "esp_err.h"

esp_err_t usb_terminal_init(void);
int usb_terminal_printf(const char *fmt, ...);
