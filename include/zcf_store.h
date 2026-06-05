// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t zcf_store_init(void);
esp_err_t zcf_store_begin(void);
esp_err_t zcf_store_begin_at(uint16_t first_block_index);
esp_err_t zcf_store_append_block(uint16_t block_index, const uint8_t *data, size_t len);
esp_err_t zcf_store_complete(void);
esp_err_t zcf_store_abort(void);
esp_err_t zcf_store_saved_size(size_t *size);
esp_err_t zcf_store_read_at(size_t offset, uint8_t *data, size_t len, size_t *bytes_read);
esp_err_t zcf_store_read_pending_at(size_t offset, uint8_t *data, size_t len, size_t *bytes_read);
size_t zcf_store_bytes_received(void);
uint16_t zcf_store_next_block_index(void);
bool zcf_store_transfer_active(void);
