// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "zcf_config.h"

#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "czone_device_identity.h"
#include "device_config.h"
#include "esp_log.h"
#include "zcf_store.h"

static const char *TAG = "zcf_config";

/* Configs seen in the field top out around 26 KB (Enigma). Reject anything far
 * larger so a corrupt size can never trigger a huge allocation, and so the byte
 * offsets below cannot overflow a 32-bit size_t. */
#define ZCF_MAX_FILE_BYTES (128U * 1024U)

/* Our board only has BOARD_RELAY_COUNT outputs, so at most a handful of circuits
 * can address us. Keep generous headroom for shared/parallelled outputs. */
#define ZCF_MAP_MAX 32

/* CZone circuit / load names are short; truncate anything longer. */
#define ZCF_NAME_MAX 40

typedef struct {
    uint8_t circuit_id;
    uint8_t relay_channel; /* 1-based, matches relay_controller_*_channel() */
    uint16_t level;        /* 0..1000 on-level from the ZCF */
    char circuit_name[ZCF_NAME_MAX]; /* label of the controlling circuit */
    char load_name[ZCF_NAME_MAX];    /* label of the physical output (load) */
} zcf_map_entry_t;

static zcf_map_entry_t s_map[ZCF_MAP_MAX];
static size_t s_map_count;
static bool s_loaded;

/* Best (largest) output-channels / "Loads" section located in the file, used to
 * resolve a channelAddress to its load label. Framing: [u32 len][u16 count][H],
 * where H is the per-record fixed-header size; each record is H header bytes
 * (channelAddress @ [0..1], nameLength @ [H-1]) followed by the name. */
static size_t s_loads_rec_start;
static size_t s_loads_end;
static uint16_t s_loads_count;
static uint8_t s_loads_h;
static bool s_loads_found;

static void copy_name(char *dst, const uint8_t *src, size_t len)
{
    if (len >= ZCF_NAME_MAX) {
        len = ZCF_NAME_MAX - 1;
    }
    memcpy(dst, src, len);
    dst[len] = '\0';
}

static inline uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t rd_u24(const uint8_t *p)
{
    return (uint32_t)(p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
}

static inline uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)(p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

/* Validate that `count` fixed-stride load records consume buf[rec_start..end)
 * exactly, and that their names look like text. */
static bool loads_section_ok(const uint8_t *buf, size_t rec_start, size_t end, uint16_t count, uint8_t h, uint16_t *out_ascii)
{
    size_t o = rec_start;
    uint16_t ascii = 0;
    for (uint16_t r = 0; r < count; ++r) {
        if (o + h > end) {
            return false;
        }
        const uint8_t nl = buf[o + h - 1];
        if (o + h + nl > end) {
            return false;
        }
        if (nl >= 2) {
            const uint8_t c = buf[o + h];
            if (c >= 0x20 && c <= 0x7E) {
                ++ascii;
            }
        }
        o = o + h + nl;
    }
    if (o != end) {
        return false;
    }
    if (out_ascii) {
        *out_ascii = ascii;
    }
    return true;
}

/* Find the Loads section (the largest valid one) and remember its bounds. */
static void find_best_loads_section(const uint8_t *buf, size_t size)
{
    s_loads_found = false;
    uint16_t best = 0;
    for (size_t i = 6; i + 8 <= size; ++i) {
        const uint32_t len = rd_u32(&buf[i]);
        if (len < 8 || len > size) {
            continue;
        }
        const size_t end = i + 4 + len;
        if (end > size) {
            continue;
        }
        const uint16_t count = rd_u16(&buf[i + 4]);
        if (count == 0 || count > 4000) {
            continue;
        }
        const uint8_t h = buf[i + 6];
        if (h < 15 || h > 24) {   /* per-channel record header, not the 40-byte device records */
            continue;
        }
        uint16_t ascii = 0;
        if (loads_section_ok(buf, i + 7, end, count, h, &ascii) && ascii >= count / 2 && count > best) {
            best = count;
            s_loads_rec_start = i + 7;
            s_loads_end = end;
            s_loads_count = count;
            s_loads_h = h;
            s_loads_found = true;
        }
    }
}

/* Resolve a channelAddress to its load label in the located Loads section. */
static void lookup_load_name(const uint8_t *buf, uint16_t channel_address, char *out)
{
    out[0] = '\0';
    if (!s_loads_found) {
        return;
    }
    size_t o = s_loads_rec_start;
    for (uint16_t r = 0; r < s_loads_count; ++r) {
        const uint16_t ca = rd_u16(&buf[o]);
        const uint8_t nl = buf[o + s_loads_h - 1];
        if (ca == channel_address) {
            copy_name(out, &buf[o + s_loads_h], nl);
            return;
        }
        o = o + s_loads_h + nl;
    }
}

/* Return the 1-based DC output (relay) number for an output channelAddress, or
 * 0 if the channel is not one of our module's DC outputs.
 *
 * This mirrors the CZone configuration tool's GetChannelString(): the DC number
 * is derived directly from the output-channel value (the low byte of the
 * channelAddress) and the mapping is module-type specific -- it is NOT the load
 * definition order. For a Control 1 / COI (module types 28 / 31) the eight DC
 * outputs occupy two channel ranges:
 *     channel 12..15  ->  DC1..DC4
 *     channel  0.. 3  ->  DC5..DC8
 * so e.g. channel 12 is DC1 and channel 0 is DC5. */
static uint8_t relay_for_channel(uint16_t channel_address)
{
    if ((channel_address >> 8) != device_config_get_dipswitch()) {
        return 0;
    }
    const uint8_t ch = (uint8_t)(channel_address & 0xFF);

    if (CZONE_DEVICE_MODULE_TYPE == 28 || CZONE_DEVICE_MODULE_TYPE == 31) {
        /* Control 1 / COI. */
        if (ch >= 12 && ch < 16) {
            return (uint8_t)(ch - 11);   /* 12->1 .. 15->4 */
        }
        if (ch < 4) {
            return (uint8_t)(ch + 5);    /* 0->5 .. 3->8 */
        }
        return 0;
    }

    /* Output Interface (type 15) and similar: DC1..DCn map 1:1 to channels. */
    if (ch < BOARD_RELAY_COUNT) {
        return (uint8_t)(ch + 1);
    }
    return 0;
}

/*
 * Walk `count` control records in buf[start..end). Each record:
 *   circuitId(1) headerFields(3 x u16) lengthOfCircuitName(1) circuitName(N)
 *   lengthOfCommandersConfig(u32) circuitCommanders(blob)
 *   lengthOfOutputConfig(u32) numberOfOutputs(u16) circuitOutputs[numberOfOutputs * 5]
 * Output entry (5 bytes): channelAddress(u16) + packed(u24); level = packed & 0x3FF.
 * Records where lengthOfOutputConfig != 2 + 5*numberOfOutputs are scenes/modes
 * (their output block references circuits, not channels) and are skipped.
 *
 * Returns true only when the records consume the section exactly, which is what
 * makes a false-positive marker effectively impossible.
 */
static bool parse_control_records(const uint8_t *buf, size_t start, size_t end, uint16_t count)
{
    size_t o = start;
    for (uint16_t r = 0; r < count; ++r) {
        if (o + 8 > end) {
            return false;
        }
        const uint8_t circuit_id = buf[o];
        const uint8_t name_len = buf[o + 7];
        const size_t name_off = o + 8;
        if (name_off + name_len + 4 > end) {
            return false;
        }

        const size_t cmd_len_off = name_off + name_len;
        const uint32_t cmd_len = rd_u32(&buf[cmd_len_off]);
        const size_t cmd_off = cmd_len_off + 4;
        if (cmd_len > (uint32_t)(end - cmd_off) || cmd_off + cmd_len + 6 > end) {
            return false;
        }

        const size_t out_len_off = cmd_off + cmd_len;
        const uint32_t out_len = rd_u32(&buf[out_len_off]);
        const uint16_t out_cnt = rd_u16(&buf[out_len_off + 4]);
        const size_t out_data = out_len_off + 6;
        if (out_len < 2 || out_len > (uint32_t)(end - (out_len_off + 4))) {
            return false;
        }
        const size_t next = out_len_off + 4 + out_len;

        if (out_len == (uint32_t)(2 + 5 * (uint32_t)out_cnt)) {
            size_t ec = out_data;
            for (uint16_t j = 0; j < out_cnt; ++j) {
                const uint16_t channel_address = rd_u16(&buf[ec]);
                const uint32_t packed = rd_u24(&buf[ec + 2]);
                if ((channel_address >> 8) == device_config_get_dipswitch()) {
                    const uint8_t relay = relay_for_channel(channel_address);
                    if (relay != 0 && s_map_count < ZCF_MAP_MAX) {
                        zcf_map_entry_t *e = &s_map[s_map_count];
                        e->circuit_id = circuit_id;
                        e->relay_channel = relay;
                        e->level = (uint16_t)(packed & 0x3FF);
                        copy_name(e->circuit_name, &buf[name_off], name_len);
                        lookup_load_name(buf, channel_address, e->load_name);
                        ++s_map_count;
                    }
                }
                ec += 5;
            }
        }
        o = next;
    }
    return o == end;
}

esp_err_t zcf_config_load(void)
{
    s_map_count = 0;
    s_loaded = false;

    size_t size = 0;
    esp_err_t err = zcf_store_saved_size(&size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no stored ZCF to parse (%s)", esp_err_to_name(err));
        return err;
    }
    if (size < 16 || size > ZCF_MAX_FILE_BYTES) {
        ESP_LOGW(TAG, "stored ZCF size %u out of range", (unsigned)size);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t *buf = malloc(size);
    if (!buf) {
        ESP_LOGE(TAG, "out of memory parsing %u-byte ZCF", (unsigned)size);
        return ESP_ERR_NO_MEM;
    }

    size_t got = 0;
    err = zcf_store_read_at(0, buf, size, &got);
    if (err != ESP_OK || got != size) {
        free(buf);
        ESP_LOGE(TAG, "failed reading stored ZCF (%s)", esp_err_to_name(err));
        return err == ESP_OK ? ESP_FAIL : err;
    }

    /* Locate the Loads (output-channels) section first so each mapping can be
     * tagged with its load label. */
    s_loads_found = false;
    find_best_loads_section(buf, size);

    /* Locate the control-records section by its `08 ?? 05 0E` marker. The byte
     * before the marker pair holds the u16 record count; six bytes before holds
     * the u32 section length. */
    bool found = false;
    for (size_t i = 6; i + 4 <= size; ++i) {
        if (buf[i] != 0x08 || buf[i + 2] != 0x05 || buf[i + 3] != 0x0E) {
            continue;
        }
        const uint32_t len = rd_u32(&buf[i - 6]);
        const uint16_t cnt = rd_u16(&buf[i - 2]);
        const size_t payload_end = (i - 2) + (size_t)len;
        if (len < 6 || cnt == 0 || payload_end > size || payload_end <= i) {
            continue;
        }
        s_map_count = 0;
        if (parse_control_records(buf, i + 4, payload_end, cnt)) {
            found = true;
            break;
        }
    }

    free(buf);
    s_loaded = found;

    if (found) {
        ESP_LOGI(TAG, "parsed ZCF: %u relay mapping(s) for dipswitch 0x%02x (%u loads)",
                 (unsigned)s_map_count, device_config_get_dipswitch(), s_loads_count);
        for (size_t i = 0; i < s_map_count; ++i) {
            ESP_LOGI(TAG, "  circuit %u '%s' -> relay %u '%s' (level %u)",
                     s_map[i].circuit_id, s_map[i].circuit_name,
                     s_map[i].relay_channel, s_map[i].load_name, s_map[i].level);
        }
        return ESP_OK;
    }

    ESP_LOGW(TAG, "no control-records section found in stored ZCF");
    return ESP_ERR_NOT_FOUND;
}

bool zcf_config_loaded(void)
{
    return s_loaded;
}

size_t zcf_config_mapping_count(void)
{
    return s_map_count;
}

uint8_t zcf_config_relays_for_circuit(uint16_t circuit_id, uint8_t *channels, uint8_t max)
{
    if (!channels || max == 0) {
        return 0;
    }
    uint8_t n = 0;
    for (size_t i = 0; i < s_map_count && n < max; ++i) {
        if (s_map[i].circuit_id == (uint8_t)circuit_id) {
            channels[n++] = s_map[i].relay_channel;
        }
    }
    return n;
}

bool zcf_config_get_entry(size_t index, uint8_t *circuit_id, uint8_t *relay_channel, uint16_t *level,
                          const char **circuit_name, const char **load_name)
{
    if (index >= s_map_count) {
        return false;
    }
    if (circuit_id) {
        *circuit_id = s_map[index].circuit_id;
    }
    if (relay_channel) {
        *relay_channel = s_map[index].relay_channel;
    }
    if (level) {
        *level = s_map[index].level;
    }
    if (circuit_name) {
        *circuit_name = s_map[index].circuit_name;
    }
    if (load_name) {
        *load_name = s_map[index].load_name;
    }
    return true;
}
