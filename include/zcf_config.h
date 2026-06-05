// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Parses the stored ZCF (/spiffs/config.zcf) and extracts the circuit -> relay
 * output mapping for THIS module only (CZONE_DEVICE_DIPSWITCH).
 *
 * A ZCF holds the configuration for every device on the CZone network. Each
 * circuit's outputs reference a channelAddress whose high byte is the target
 * module's dipswitch and whose low byte is that module's output channel:
 *
 *     channelAddress = (moduleDipswitch << 8) | outputChannel
 *
 * We keep only the outputs addressed to our dipswitch, turning each into a
 * 1-based physical relay channel (outputChannel + 1).
 *
 * The control-records section layout was reverse-engineered from
 * configuration-tool.exe (DecodeZcfBinaryToConfigModel / AppendCircuitsToJson /
 * AppendCircuitOutputToJson) and validated against real configs.
 */

/* Load + parse the committed config.zcf. Safe to call repeatedly (replaces the
 * previous mapping). Returns ESP_OK when a control-records section was parsed,
 * ESP_ERR_NOT_FOUND when no config / no section is present. */
esp_err_t zcf_config_load(void);

/* True once a config has been parsed successfully. */
bool zcf_config_loaded(void);

/* Number of (circuit, relay) output mappings held for this module. */
size_t zcf_config_mapping_count(void);

/* Fill `channels` with the 1-based relay channels driven by `circuit_id` on this
 * module. Returns the number written (0 if the circuit drives none of our
 * outputs). */
uint8_t zcf_config_relays_for_circuit(uint16_t circuit_id, uint8_t *channels, uint8_t max);

/* Read back the i-th mapping entry (for diagnostics / terminal dump). The name
 * pointers reference internal storage valid until the next zcf_config_load().
 * Any out-pointer may be NULL. Returns false when `index` is out of range. */
bool zcf_config_get_entry(size_t index, uint8_t *circuit_id, uint8_t *relay_channel, uint16_t *level,
                          const char **circuit_name, const char **load_name);
