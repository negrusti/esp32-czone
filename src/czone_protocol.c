// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "czone_protocol.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "board.h"
#include "czone_device_identity.h"
#include "device_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "relay_controller.h"
#include "usb_terminal.h"
#include "zcf_config.h"
#include "zcf_store.h"

static const char *TAG = "czone_pgn";

#define N2K_PRIORITY_DEFAULT        6
#define N2K_GLOBAL_ADDRESS          0xff

#define PGN_ISO_ADDRESS_CLAIM       60928U
#define PGN_ISO_REQUEST             59904U
#define PGN_CZONE_SS_COMMAND        65280U
#define PGN_CZONE_SS_FEEDBACK       65281U
#define PGN_CZONE_STATUS            65284U
#define PGN_CZONE_BRIEF_STATUS      65288U
#define PGN_CZONE_CONFIG_CLAIM      65290U
#define PGN_CZONE_DATABLOCK_ACK     65291U
#define PGN_CZONE_CAL_REQUEST       65293U
#define PGN_CZONE_BACKLIGHT         65294U
#define PGN_CZONE_ALARM             65295U
#define PGN_CZONE_STATUS_EXT_SC     65301U
#define PGN_BEP_SWITCH_GROUP        65308U
#define PGN_BEP_SWITCH_INSTANCE     65309U
#define PGN_BEP_ANALOGUE_VALUE      65310U
#define PGN_CZONE_DATABLOCK         130816U
#define PGN_CZONE_OI_STATUS         130817U
#define PGN_CZONE_DEBUG             130821U
#define PGN_CZONE_ZONE_CAL          130822U
#define PGN_PGN_LIST                126464U
#define PGN_HEARTBEAT               126993U
#define PGN_PRODUCT_INFO            126996U
#define PGN_CONFIG_INFO             126998U
#define PGN_SWITCH_BANK_STATUS      127501U
#define PGN_SWITCH_BANK_CONTROL     127502U

#define SWITCHES_PER_BANK           28
#define FAST_PACKET_MAX_PAYLOAD     223
#define FAST_PACKET_SESSIONS        8
#define FAST_PACKET_SESSION_TTL_MS  2000U
#define HEARTBEAT_INTERVAL_MS       60000U
#define CZONE_STATUS_INTERVAL_MS    1000U
#define ADDRESS_CLAIM_MAX_ADDRESS   253U
#define ZCF_DATABLOCK_HEADER_LEN    23U
#define ZCF_DATABLOCK_CHUNK_MAX     200U
#define ZCF_SEND_ACK_TIMEOUT_MS     750U
#define ZCF_SEND_MAX_RETRIES        8U
#define ZCF_SEND_TRIGGER_DELAY_MS   750U
#define CZONE_SS_CMD_PRESS          0xf1U
#define CZONE_SS_CMD_PRESS_ALT      0xf2U
#define CZONE_SS_CMD_RELEASE        0x40U
#define CZONE_OI_STATUS_PAGE        0x01U
#define CZONE_OI_STATUS_RECORDS     6U
#define CZONE_OI_STATUS_RECORD_LEN  3U
#define PROTOCOL_NOTIFY_ZCF_SEND    (1UL << 0)

typedef struct {
    uint32_t pgn;
    uint8_t src;
    uint8_t dst;
    uint8_t priority;
} n2k_id_t;

typedef struct {
    bool active;
    uint32_t pgn;
    uint8_t src;
    uint8_t seq;
    uint8_t total_len;
    uint8_t received;
    uint8_t next_frame;
    uint8_t expected_frames;
    uint32_t frame_mask;
    TickType_t updated_at;
    uint16_t zcf_subcode;
    uint8_t zcf_flag;
    uint8_t data[FAST_PACKET_MAX_PAYLOAD];
} fast_packet_session_t;

static fast_packet_session_t s_fp[FAST_PACKET_SESSIONS];
static czone_can_monitor_mode_t s_monitor_mode = CZONE_CAN_MONITOR_OFF;
static uint8_t s_source_address = CZONE_DEVICE_PREFERRED_ADDRESS;
static uint64_t s_n2k_name;
static uint8_t s_factory_mac[6];
static uint8_t s_fp_tx_seq;
static uint8_t s_heartbeat_seq;
static bool s_started;
static TaskHandle_t s_protocol_task_handle;
/* Config-send (read) arbitration. Every CZone module holds the config, so when
 * the tool requests a read the holders must elect a single sender. Mirroring
 * configuration-tool.exe (CZone_ZCF_TransferStateMachine state 2 waits
 * (moduleAddr>>3)+20 ticks before claiming; sub_582360 makes a higher-address
 * holder defer when a lower-address holder claims), each holder backs off
 * proportional to its module address and defers to any lower-address claimant.
 * Lowest module address wins. */
#define ZCF_ARB_TICK_MS 10
static volatile bool s_zcf_send_requested;
static volatile bool s_zcf_send_active;
static volatile bool s_zcf_arb_pending;
static volatile TickType_t s_zcf_arb_deadline;
/* While set in the future, the protocol task keeps broadcasting a module
 * config-claim so the tool registers us as a write responder (and sends
 * block 0). Set when a network-write announce is received. */
static volatile TickType_t s_module_claim_until;
static SemaphoreHandle_t s_zcf_send_ack_sem;
static volatile bool s_zcf_send_waiting_ack;
static volatile uint8_t s_zcf_send_ack_src;
static volatile uint8_t s_zcf_send_ack_status;
static volatile uint16_t s_zcf_send_ack_block;

static const uint32_t s_tx_pgns[] = {
    PGN_ISO_ADDRESS_CLAIM,
    PGN_PGN_LIST,
    PGN_HEARTBEAT,
    PGN_PRODUCT_INFO,
    PGN_CONFIG_INFO,
    PGN_SWITCH_BANK_STATUS,
    PGN_CZONE_STATUS,
    PGN_CZONE_BRIEF_STATUS,
    PGN_CZONE_OI_STATUS,
    PGN_CZONE_SS_FEEDBACK,
    PGN_CZONE_DATABLOCK,
    PGN_CZONE_DATABLOCK_ACK,
};

static const uint32_t s_rx_pgns[] = {
    PGN_ISO_ADDRESS_CLAIM,
    PGN_ISO_REQUEST,
    PGN_SWITCH_BANK_CONTROL,
    PGN_CZONE_SS_COMMAND,
    PGN_CZONE_CONFIG_CLAIM,
    PGN_CZONE_DATABLOCK,
    PGN_CZONE_DATABLOCK_ACK,
    PGN_CZONE_CAL_REQUEST,
    PGN_CZONE_BACKLIGHT,
};

static esp_err_t send_pgn_lists(void);
static esp_err_t send_heartbeat(void);
static esp_err_t send_czone_module_status(uint32_t module_specific_bits);
static esp_err_t send_czone_oi_status(uint8_t relay_mask);
static esp_err_t send_switch_bank_status(uint8_t relay_mask);
static esp_err_t send_czone_zcf_size_claim(size_t file_size);
static esp_err_t send_czone_module_claim(void);
static esp_err_t handle_config_claim(uint8_t src, const uint8_t *data, uint8_t len);
static esp_err_t handle_zcf_datablock_ack(uint8_t src, const uint8_t *data, uint8_t len);
static void expire_stale_fast_packet_sessions(TickType_t now);

static n2k_id_t n2k_decode(uint32_t can_id)
{
    n2k_id_t id = {
        .priority = (uint8_t)((can_id >> 26) & 0x07),
        .src = (uint8_t)(can_id & 0xff),
        .dst = N2K_GLOBAL_ADDRESS,
    };
    const uint32_t dp = (can_id >> 24) & 0x01;
    const uint32_t pf = (can_id >> 16) & 0xff;
    const uint32_t ps = (can_id >> 8) & 0xff;

    if (pf >= 240) {
        id.pgn = (dp << 16) | (pf << 8) | ps;
    } else {
        id.pgn = (dp << 16) | (pf << 8);
        id.dst = (uint8_t)ps;
    }
    return id;
}

static uint32_t n2k_build_can_id(uint32_t pgn, uint8_t src, uint8_t priority)
{
    const uint32_t dp = (pgn >> 16) & 0x01;
    const uint32_t pf = (pgn >> 8) & 0xff;
    uint32_t ps = pgn & 0xff;

    if (pf < 240) {
        ps = N2K_GLOBAL_ADDRESS;
    }

    return ((uint32_t)priority << 26) | (dp << 24) | (pf << 16) | (ps << 8) | src;
}

static uint64_t build_n2k_name(void)
{
    uint32_t identity = CZONE_DEVICE_IDENTITY_NUMBER;
    if (identity == 0) {
        if (s_factory_mac[0] || s_factory_mac[1] || s_factory_mac[2] ||
            s_factory_mac[3] || s_factory_mac[4] || s_factory_mac[5]) {
            identity = (((uint32_t)s_factory_mac[3] << 16) |
                        ((uint32_t)s_factory_mac[4] << 8) |
                        s_factory_mac[5]) & 0x1fffffU;
        }
        if (identity == 0) {
            identity = 1;
        }
    }

    uint64_t name = 0;
    name |= ((uint64_t)identity & 0x1fffffULL);
    name |= (((uint64_t)CZONE_DEVICE_PRODUCT_MANUFACTURER_CODE & 0x07ffULL) << 21);
    /* Device Instance (bits 32..39: lower 3 + upper 5). The CZone tool shows
     * this as the module "Dipswitch" (lower/upper (combined)), so publish our
     * module address here -- otherwise the tool lists it as 0/0 (0). */
    name |= (((uint64_t)device_config_get_dipswitch() & 0xffULL) << 32);
    name |= (((uint64_t)CZONE_DEVICE_FUNCTION & 0xffULL) << 40);
    name |= (((uint64_t)CZONE_DEVICE_CLASS & 0x7fULL) << 50);
    name |= (((uint64_t)CZONE_DEVICE_INDUSTRY_GROUP & 0x0fULL) << 60);
    name |= (1ULL << 63);
    return name;
}

static void write_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xff);
    dst[1] = (uint8_t)(value >> 8);
}

static uint16_t read_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

/* CRC-8 used inside ZCF files: poly 0x07, init 0, no reflection (MSB-first),
 * no final XOR. Matches the CZone tool and the ZcfReverse encoder. */
static uint8_t zcf_crc8_update(uint8_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t czone_vendor_header(void)
{
    return (uint16_t)((CZONE_DEVICE_MANUFACTURER_CODE & 0x07ff) |
                      (0x03 << 11) |
                      ((CZONE_DEVICE_INDUSTRY_GROUP & 0x07) << 13));
}

static void write_u64_le(uint8_t *dst, uint64_t value)
{
    for (uint8_t i = 0; i < 8; ++i) {
        dst[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint64_t read_u64_le(const uint8_t *src)
{
    uint64_t value = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        value |= ((uint64_t)src[i]) << (8U * i);
    }
    return value;
}

static void write_fixed_ascii(uint8_t *dst, size_t len, const char *text)
{
    memset(dst, 0, len);
    for (size_t i = 0; i < len && text[i] != '\0'; ++i) {
        dst[i] = (uint8_t)text[i];
    }
}

static void format_serial_code(char *dst, size_t len)
{
    snprintf(dst, len, "%02X%02X%02X%02X%02X%02X",
             s_factory_mac[0], s_factory_mac[1], s_factory_mac[2],
             s_factory_mac[3], s_factory_mac[4], s_factory_mac[5]);
}

static uint8_t append_lau_ascii(uint8_t *dst, uint8_t offset, uint8_t max_len, const char *text)
{
    uint8_t content_len = 0;
    while (text[content_len] != '\0' && content_len < 0xfd && (uint16_t)offset + 2U + content_len < max_len) {
        ++content_len;
    }

    dst[offset++] = (uint8_t)(content_len + 2U);
    dst[offset++] = 0x01;
    for (uint8_t i = 0; i < content_len; ++i) {
        dst[offset++] = (uint8_t)text[i];
    }
    return offset;
}

static const char *pgn_name(uint32_t pgn)
{
    switch (pgn) {
    case PGN_ISO_ADDRESS_CLAIM: return "ISO Address Claim";
    case PGN_ISO_REQUEST: return "ISO Request";
    case PGN_CZONE_SS_COMMAND: return "CZone SS Command";
    case PGN_CZONE_SS_FEEDBACK: return "CZone SS Feedback";
    case PGN_CZONE_STATUS: return "CZone Status";
    case PGN_CZONE_BRIEF_STATUS: return "CZone Brief Status";
    case PGN_CZONE_CONFIG_CLAIM: return "CZone Config Claim";
    case PGN_CZONE_DATABLOCK_ACK: return "CZone DataBlock ACK";
    case PGN_CZONE_CAL_REQUEST: return "CZone Cal Request";
    case PGN_CZONE_BACKLIGHT: return "CZone Backlight";
    case PGN_CZONE_ALARM: return "CZone Alarm";
    case PGN_CZONE_STATUS_EXT_SC: return "CZone Status Ext SC";
    case PGN_BEP_SWITCH_GROUP: return "BEP Switch Group";
    case PGN_BEP_SWITCH_INSTANCE: return "BEP Switch Instance";
    case PGN_BEP_ANALOGUE_VALUE: return "BEP Analogue Value";
    case PGN_CZONE_DATABLOCK: return "CZone DataBlock";
    case PGN_CZONE_OI_STATUS: return "CZone OI Status";
    case PGN_CZONE_DEBUG: return "CZone Debug";
    case PGN_CZONE_ZONE_CAL: return "CZone Zone Cal";
    case PGN_PGN_LIST: return "PGN List";
    case PGN_HEARTBEAT: return "Heartbeat";
    case PGN_PRODUCT_INFO: return "Product Info";
    case PGN_CONFIG_INFO: return "Config Info";
    case PGN_SWITCH_BANK_STATUS: return "Switch Bank Status";
    case PGN_SWITCH_BANK_CONTROL: return "Switch Bank Control";
    default: return "unknown";
    }
}

static bool is_czone_proprietary_pgn(uint32_t pgn)
{
    return (pgn >= 65280U && pgn <= 65310U) ||
           pgn == PGN_CZONE_DATABLOCK ||
           pgn == PGN_CZONE_OI_STATUS ||
           pgn == PGN_CZONE_DEBUG ||
           pgn == PGN_CZONE_ZONE_CAL;
}

static bool is_czone_standard_pgn(uint32_t pgn)
{
    return pgn == PGN_SWITCH_BANK_STATUS ||
           pgn == PGN_SWITCH_BANK_CONTROL;
}

static bool is_fast_packet_pgn(uint32_t pgn)
{
    return pgn == PGN_PGN_LIST ||
           pgn == PGN_PRODUCT_INFO ||
           pgn == PGN_CONFIG_INFO ||
           pgn == PGN_CZONE_DATABLOCK ||
           pgn == PGN_CZONE_OI_STATUS ||
           pgn == PGN_CZONE_DEBUG ||
           pgn == PGN_CZONE_ZONE_CAL;
}

static bool has_czone_vendor_header(const uint8_t *data, uint8_t len)
{
    if (len < 2) {
        return false;
    }

    const uint16_t header = read_u16_le(data);
    return (header & 0x07ff) == CZONE_DEVICE_MANUFACTURER_CODE &&
           ((header >> 13) & 0x07) == CZONE_DEVICE_INDUSTRY_GROUP;
}

static bool is_czone_pgn(uint32_t pgn, const uint8_t *data, uint8_t len)
{
    if (is_czone_standard_pgn(pgn)) {
        return true;
    }
    if (is_czone_proprietary_pgn(pgn)) {
        return has_czone_vendor_header(data, len);
    }
    return false;
}

void czone_protocol_set_monitor_mode(czone_can_monitor_mode_t mode)
{
    if (mode <= CZONE_CAN_MONITOR_ALL) {
        s_monitor_mode = mode;
    }
}

czone_can_monitor_mode_t czone_protocol_get_monitor_mode(void)
{
    return s_monitor_mode;
}

const char *czone_protocol_monitor_mode_name(czone_can_monitor_mode_t mode)
{
    switch (mode) {
    case CZONE_CAN_MONITOR_OFF: return "off";
    case CZONE_CAN_MONITOR_CZONE: return "czone";
    case CZONE_CAN_MONITOR_ZCF: return "zcf";
    case CZONE_CAN_MONITOR_ALL: return "all";
    default: return "unknown";
    }
}

static bool fast_packet_key_matches(const fast_packet_session_t *slot, uint32_t pgn, uint8_t src, uint8_t seq)
{
    return slot->active && slot->pgn == pgn && slot->src == src && slot->seq == seq;
}

static void expire_stale_fast_packet_sessions(TickType_t now)
{
    const TickType_t ttl = pdMS_TO_TICKS(FAST_PACKET_SESSION_TTL_MS);
    for (uint8_t i = 0; i < FAST_PACKET_SESSIONS; ++i) {
        fast_packet_session_t *slot = &s_fp[i];
        if (!slot->active || (now - slot->updated_at) < ttl) {
            continue;
        }
        slot->active = false;
    }
}

static bool is_zcf_transfer_pgn(uint32_t pgn)
{
    return pgn == PGN_CZONE_CONFIG_CLAIM ||
           pgn == PGN_CZONE_DATABLOCK ||
           pgn == PGN_CZONE_DATABLOCK_ACK;
}

static bool should_log_pgn(uint32_t pgn, const uint8_t *data, uint8_t len)
{
    if (s_monitor_mode == CZONE_CAN_MONITOR_ALL) {
        return true;
    }
    if (s_monitor_mode == CZONE_CAN_MONITOR_ZCF) {
        return is_zcf_transfer_pgn(pgn);
    }
    if (s_monitor_mode == CZONE_CAN_MONITOR_CZONE) {
        return is_czone_pgn(pgn, data, len);
    }
    return false;
}

/* Prints a "[<ms>.<us> +<delta_ms>.<us>] " timestamp prefix for CAN log lines:
 * absolute time since boot plus the gap since the previous logged frame, so the
 * raw fast-packet cadence and inter-block timing are visible. */
static void log_timestamp(void)
{
    static int64_t last_us = 0;
    const int64_t now_us = esp_timer_get_time();
    const int64_t delta_us = last_us != 0 ? (now_us - last_us) : 0;
    last_us = now_us;
    usb_terminal_printf("[%lu.%03lu +%lu.%03lu] ",
                        (unsigned long)(now_us / 1000), (unsigned long)(now_us % 1000),
                        (unsigned long)(delta_us / 1000), (unsigned long)(delta_us % 1000));
}

static void print_data(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0; i < len; ++i) {
        usb_terminal_printf("%02x%s", data[i], i + 1 == len ? "" : " ");
    }
}

static void log_frame(const czone_can_frame_t *frame, const n2k_id_t *id)
{
    if (!should_log_pgn(id->pgn, frame->data, frame->dlc)) {
        return;
    }

    /* Only ALL mode dumps every raw frame of a multi-frame PGN; czone and zcf
     * keep it to the single reassembled line (see log_reassembled). */
    if (is_fast_packet_pgn(id->pgn) && s_monitor_mode != CZONE_CAN_MONITOR_ALL) {
        return;
    }

    log_timestamp();
    if (s_monitor_mode == CZONE_CAN_MONITOR_ALL && !is_czone_pgn(id->pgn, frame->data, frame->dlc)) {
        usb_terminal_printf("[CAN RX] id=0x%08lx PGN %lu (0x%05lx %s) src=%u dst=%u pri=%u len=%u data=",
               frame->can_id, id->pgn, id->pgn, pgn_name(id->pgn), id->src, id->dst, id->priority, frame->dlc);
    } else {
        usb_terminal_printf("[CZone RX] PGN %lu (0x%05lx %s) src=%u dst=%u len=%u data=",
               id->pgn, id->pgn, pgn_name(id->pgn), id->src, id->dst, frame->dlc);
    }
    print_data(frame->data, frame->dlc);
    usb_terminal_printf("\r\n");
}

static void log_tx_frame(const czone_can_frame_t *frame)
{
    const n2k_id_t id = n2k_decode(frame->can_id);
    if (!should_log_pgn(id.pgn, frame->data, frame->dlc)) {
        return;
    }

    /* Only ALL mode dumps every raw frame of a multi-frame PGN. */
    if (is_fast_packet_pgn(id.pgn) && s_monitor_mode != CZONE_CAN_MONITOR_ALL) {
        return;
    }

    log_timestamp();
    if (s_monitor_mode == CZONE_CAN_MONITOR_ALL && !is_czone_pgn(id.pgn, frame->data, frame->dlc)) {
        usb_terminal_printf("[CAN TX] id=0x%08lx PGN %lu (0x%05lx %s) src=%u dst=%u pri=%u len=%u data=",
                            frame->can_id, id.pgn, id.pgn, pgn_name(id.pgn), id.src, id.dst, id.priority, frame->dlc);
    } else {
        usb_terminal_printf("[CZone TX] PGN %lu (0x%05lx %s) src=%u dst=%u len=%u data=",
                            id.pgn, id.pgn, pgn_name(id.pgn), id.src, id.dst, frame->dlc);
    }
    print_data(frame->data, frame->dlc);
    usb_terminal_printf("\r\n");
}

static esp_err_t send_pgn_priority(uint32_t pgn, uint8_t priority, const uint8_t *payload, uint8_t len)
{
    ESP_RETURN_ON_FALSE(len <= 8, ESP_ERR_INVALID_SIZE, TAG, "single-frame PGN too large");

    czone_can_frame_t frame = {
        .can_id = n2k_build_can_id(pgn, s_source_address, priority),
        .extended = true,
        .dlc = 8,
    };

    memset(frame.data, 0xff, sizeof(frame.data));
    memcpy(frame.data, payload, len);
    const esp_err_t err = czone_can_send(&frame);
    if (err == ESP_OK) {
        log_tx_frame(&frame);
    }
    return err;
}

static esp_err_t send_pgn(uint32_t pgn, const uint8_t *payload, uint8_t len)
{
    return send_pgn_priority(pgn, N2K_PRIORITY_DEFAULT, payload, len);
}

static esp_err_t send_fast_pgn(uint32_t pgn, const uint8_t *payload, uint8_t len)
{
    ESP_RETURN_ON_FALSE(len <= FAST_PACKET_MAX_PAYLOAD, ESP_ERR_INVALID_SIZE, TAG, "fast-packet too large");

    const uint8_t seq = s_fp_tx_seq++ & 0x07;
    czone_can_frame_t frame = {
        .can_id = n2k_build_can_id(pgn, s_source_address, N2K_PRIORITY_DEFAULT),
        .extended = true,
        .dlc = 8,
    };

    memset(frame.data, 0xff, sizeof(frame.data));
    frame.data[0] = (uint8_t)(seq << 5);
    frame.data[1] = len;

    uint8_t offset = 0;
    uint8_t take = len < 6 ? len : 6;
    memcpy(&frame.data[2], payload, take);
    offset += take;
    ESP_RETURN_ON_ERROR(czone_can_send(&frame), TAG, "send fast-packet first frame");
    log_tx_frame(&frame);

    for (uint8_t frame_num = 1; offset < len; ++frame_num) {
        memset(frame.data, 0xff, sizeof(frame.data));
        frame.data[0] = (uint8_t)((seq << 5) | (frame_num & 0x1f));
        take = (uint8_t)((len - offset) < 7 ? (len - offset) : 7);
        memcpy(&frame.data[1], &payload[offset], take);
        offset += take;
        ESP_RETURN_ON_ERROR(czone_can_send(&frame), TAG, "send fast-packet continuation");
        log_tx_frame(&frame);
    }

    return ESP_OK;
}

static esp_err_t send_address_claim(void)
{
    uint8_t payload[8];
    write_u64_le(payload, s_n2k_name);
    ESP_LOGI(TAG, "claiming N2K address %u NAME=0x%08lx%08lx",
             s_source_address, (uint32_t)(s_n2k_name >> 32), (uint32_t)s_n2k_name);
    return send_pgn(PGN_ISO_ADDRESS_CLAIM, payload, sizeof(payload));
}

static esp_err_t send_product_information(void)
{
    uint8_t payload[134];
    char serial_code[13];
    memset(payload, 0, sizeof(payload));
    format_serial_code(serial_code, sizeof(serial_code));

    write_u16_le(&payload[0], CZONE_DEVICE_N2K_DB_VERSION);
    write_u16_le(&payload[2], CZONE_DEVICE_PRODUCT_CODE);
    write_fixed_ascii(&payload[4], 32, CZONE_DEVICE_MODEL_ID);
    write_fixed_ascii(&payload[36], 32, CZONE_DEVICE_SOFTWARE_VERSION);
    write_fixed_ascii(&payload[68], 32, CZONE_DEVICE_MODEL_VERSION);
    write_fixed_ascii(&payload[100], 32, serial_code);
    payload[132] = 1;
    payload[133] = 1;

    return send_fast_pgn(PGN_PRODUCT_INFO, payload, sizeof(payload));
}

static esp_err_t send_config_information(void)
{
    uint8_t payload[96];
    uint8_t len = 0;

    len = append_lau_ascii(payload, len, sizeof(payload), "");
    len = append_lau_ascii(payload, len, sizeof(payload), "");
    len = append_lau_ascii(payload, len, sizeof(payload), CZONE_DEVICE_MANUFACTURER_INFO);

    return send_fast_pgn(PGN_CONFIG_INFO, payload, len);
}

static void handle_address_claim(const n2k_id_t *id, const uint8_t *data, uint8_t len)
{
    if (len < 8 || id->src != s_source_address) {
        return;
    }

    const uint64_t their_name = read_u64_le(data);
    if (their_name == s_n2k_name) {
        return;
    }

    if (their_name < s_n2k_name) {
        if (s_source_address < ADDRESS_CLAIM_MAX_ADDRESS) {
            ++s_source_address;
            ESP_LOGW(TAG, "address conflict lost; moving to N2K address %u", s_source_address);
        } else {
            ESP_LOGE(TAG, "address conflict lost and no free address remains");
        }
    } else {
        ESP_LOGI(TAG, "address conflict won; reclaiming N2K address %u", s_source_address);
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(send_address_claim());
}

static void protocol_task(void *arg)
{
    (void)arg;
    TickType_t last_heartbeat = xTaskGetTickCount();
    TickType_t last_status = last_heartbeat;

    ESP_ERROR_CHECK_WITHOUT_ABORT(send_address_claim());
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_pgn_lists());
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_product_information());
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_config_information());
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_czone_module_status(relay_controller_get_mask()));
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_czone_oi_status(relay_controller_get_mask()));
    ESP_ERROR_CHECK_WITHOUT_ABORT(send_switch_bank_status(relay_controller_get_mask()));

    while (true) {
        /* Wake no later than a pending arbitration back-off so we can send the
         * moment we win. */
        TickType_t wait_ticks = pdMS_TO_TICKS(CZONE_STATUS_INTERVAL_MS);
        if (s_zcf_arb_pending) {
            const int32_t remain = (int32_t)(s_zcf_arb_deadline - xTaskGetTickCount());
            if (remain <= 0) {
                wait_ticks = 0;
            } else if ((TickType_t)remain < wait_ticks) {
                wait_ticks = (TickType_t)remain;
            }
        }
        uint32_t notify_bits = 0;
        xTaskNotifyWait(0, UINT32_MAX, &notify_bits, wait_ticks);
        (void)notify_bits;

        /* Won the config-send arbitration: the back-off elapsed and no
         * lower-address holder claimed, so transmit our stored config. */
        if (s_zcf_arb_pending && (int32_t)(xTaskGetTickCount() - s_zcf_arb_deadline) >= 0) {
            s_zcf_arb_pending = false;
            s_zcf_send_requested = true;
        }

        if (s_zcf_send_requested) {
            vTaskDelay(pdMS_TO_TICKS(ZCF_SEND_TRIGGER_DELAY_MS));
            if (s_zcf_send_requested) {
                s_zcf_send_requested = false;
                const esp_err_t err = czone_protocol_send_zcf();
                if (err == ESP_OK) {
                    usb_terminal_printf("[CZone ZCF] stored ZCF sent\r\n");
                } else {
                    usb_terminal_printf("[CZone ZCF] send failed: %s\r\n", esp_err_to_name(err));
                }
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_status) >= pdMS_TO_TICKS(CZONE_STATUS_INTERVAL_MS)) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_czone_module_status(relay_controller_get_mask()));
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_czone_oi_status(relay_controller_get_mask()));
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_switch_bank_status(relay_controller_get_mask()));
            last_status = now;
        }
        /* During a network-write window keep advertising ourselves so the tool
         * registers us as a responder and transmits block 0. */
        if ((int32_t)(s_module_claim_until - now) > 0) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_czone_module_claim());
        }
        if ((now - last_heartbeat) >= pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS)) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(send_heartbeat());
            last_heartbeat = now;
        }
    }
}

esp_err_t czone_protocol_init(void)
{
    if (s_started) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(zcf_store_init(), TAG, "init ZCF store");
    s_zcf_send_ack_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_zcf_send_ack_sem != NULL, ESP_ERR_NO_MEM, TAG, "create ZCF ACK semaphore");
    ESP_RETURN_ON_ERROR(esp_efuse_mac_get_default(s_factory_mac), TAG, "read factory MAC");
    s_n2k_name = build_n2k_name();
    BaseType_t ok = xTaskCreate(protocol_task, "czone_protocol", 4096, NULL, 5, &s_protocol_task_handle);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create protocol task");
    s_started = true;
    return ESP_OK;
}

static uint8_t *feed_fast_packet(uint32_t can_id, uint32_t pgn, uint8_t src, const uint8_t *data, uint8_t len, uint8_t *out_len)
{
    if (len < 2) {
        return NULL;
    }

    const TickType_t now = xTaskGetTickCount();
    const uint8_t header = data[0];
    const uint8_t seq = (header >> 5) & 0x07;
    const uint8_t frame_num = header & 0x1f;
    fast_packet_session_t *slot = NULL;
    (void)can_id;
    expire_stale_fast_packet_sessions(now);

    if (frame_num == 0) {
        for (uint8_t i = 0; i < FAST_PACKET_SESSIONS; ++i) {
            if (fast_packet_key_matches(&s_fp[i], pgn, src, seq)) {
                slot = &s_fp[i];
                break;
            }
        }
        for (uint8_t i = 0; i < FAST_PACKET_SESSIONS; ++i) {
            if (!slot && !s_fp[i].active) {
                slot = &s_fp[i];
                break;
            }
        }
        if (!slot) {
            uint8_t oldest = 0;
            for (uint8_t i = 1; i < FAST_PACKET_SESSIONS; ++i) {
                if (s_fp[i].updated_at < s_fp[oldest].updated_at) {
                    oldest = i;
                }
            }
            slot = &s_fp[oldest];
        }

        memset(slot, 0, sizeof(*slot));
        slot->active = true;
        slot->pgn = pgn;
        slot->src = src;
        slot->seq = seq;
        slot->total_len = data[1];
        if (slot->total_len > FAST_PACKET_MAX_PAYLOAD) {
            slot->active = false;
            return NULL;
        }

        const uint8_t first_data_len = (uint8_t)(len - 2U);
        const uint8_t copy = slot->total_len < 6 ? slot->total_len : 6;
        if (first_data_len < copy) {
            slot->active = false;
            return NULL;
        }
        memcpy(slot->data, &data[2], copy);
        slot->received = copy;
        slot->next_frame = 1;
        slot->expected_frames = slot->total_len <= 6
            ? 1
            : (uint8_t)(1U + ((slot->total_len - 6U) + 6U) / 7U);
        slot->frame_mask = 1U;
        slot->updated_at = now;
        slot->zcf_subcode = copy >= 4 ? read_u16_le(&slot->data[2]) : 0xffff;
        slot->zcf_flag = copy >= 5 ? slot->data[4] : 0xff;
        if (slot->received >= slot->total_len) {
            slot->active = false;
            *out_len = slot->total_len;
            return slot->data;
        }
        return NULL;
    }

    for (uint8_t i = 0; i < FAST_PACKET_SESSIONS; ++i) {
        if (fast_packet_key_matches(&s_fp[i], pgn, src, seq)) {
            slot = &s_fp[i];
            break;
        }
    }
    if (!slot) {
        return NULL;
    }
    if (frame_num < 32 && (slot->frame_mask & (1UL << frame_num)) != 0) {
        slot->updated_at = now;
        return NULL;
    }

    const uint16_t dest = 6U + ((uint16_t)frame_num - 1U) * 7U;
    if (frame_num >= slot->expected_frames || dest >= slot->total_len || dest >= FAST_PACKET_MAX_PAYLOAD) {
        slot->active = false;
        return NULL;
    }

    const uint8_t frame_data_len = (uint8_t)(len - 1U);
    const uint8_t remaining = (uint8_t)(slot->total_len - dest);
    const uint8_t copy = remaining < 7 ? remaining : 7;
    if (copy == 0 || frame_data_len < copy) {
        slot->active = false;
        return NULL;
    }
    memcpy(&slot->data[dest], &data[1], copy);
    const uint8_t received_end = (uint8_t)(dest + copy);
    if (received_end > slot->received) {
        slot->received = received_end;
    }
    slot->frame_mask |= (1UL << frame_num);
    slot->updated_at = now;
    while (slot->next_frame < slot->expected_frames &&
           (slot->frame_mask & (1UL << slot->next_frame)) != 0) {
        ++slot->next_frame;
    }

    const uint32_t required_mask = slot->expected_frames >= 32
        ? 0xffffffffUL
        : ((1UL << slot->expected_frames) - 1UL);
    if ((slot->frame_mask & required_mask) == required_mask) {
        slot->active = false;
        *out_len = slot->total_len;
        return slot->data;
    }

    return NULL;
}

static void build_switch_bank_status(uint8_t relay_mask, uint8_t payload[8])
{
    memset(payload, 0, 8);
    payload[0] = 0;

    for (uint8_t sw = 0; sw < SWITCHES_PER_BANK; ++sw) {
        const uint8_t bits = sw < BOARD_RELAY_COUNT
            ? ((relay_mask & (uint8_t)(1U << sw)) ? 1U : 0U)
            : 3U;
        payload[1 + sw / 4] |= (uint8_t)(bits << ((sw % 4) * 2));
    }
}

static esp_err_t send_switch_bank_status(uint8_t relay_mask)
{
    uint8_t bank[8] = {0};
    build_switch_bank_status(relay_mask, bank);
    return send_pgn(PGN_SWITCH_BANK_STATUS, bank, sizeof(bank));
}

static esp_err_t send_pgn_lists(void)
{
    uint8_t payload[1 + 3 * 16];
    uint8_t len = 1;

    payload[0] = 0;
    for (uint8_t i = 0; i < sizeof(s_tx_pgns) / sizeof(s_tx_pgns[0]); ++i) {
        payload[len++] = (uint8_t)(s_tx_pgns[i] & 0xff);
        payload[len++] = (uint8_t)((s_tx_pgns[i] >> 8) & 0xff);
        payload[len++] = (uint8_t)((s_tx_pgns[i] >> 16) & 0xff);
    }
    ESP_RETURN_ON_ERROR(send_fast_pgn(PGN_PGN_LIST, payload, len), TAG, "send TX PGN list");

    len = 1;
    payload[0] = 1;
    for (uint8_t i = 0; i < sizeof(s_rx_pgns) / sizeof(s_rx_pgns[0]); ++i) {
        payload[len++] = (uint8_t)(s_rx_pgns[i] & 0xff);
        payload[len++] = (uint8_t)((s_rx_pgns[i] >> 8) & 0xff);
        payload[len++] = (uint8_t)((s_rx_pgns[i] >> 16) & 0xff);
    }
    return send_fast_pgn(PGN_PGN_LIST, payload, len);
}

static esp_err_t send_heartbeat(void)
{
    const uint16_t interval_10ms = 6000;
    const uint8_t payload[8] = {
        (uint8_t)(interval_10ms & 0xff),
        (uint8_t)(interval_10ms >> 8),
        s_heartbeat_seq,
        0b11001100,
        0xff, 0xff, 0xff, 0xff,
    };
    s_heartbeat_seq = s_heartbeat_seq >= 252 ? 0 : (uint8_t)(s_heartbeat_seq + 1);
    return send_pgn(PGN_HEARTBEAT, payload, sizeof(payload));
}

static esp_err_t send_czone_module_status(uint32_t module_specific_bits)
{
    uint8_t payload[8] = {0};
    write_u16_le(payload, czone_vendor_header());
    payload[2] = device_config_get_dipswitch();
    payload[3] = CZONE_DEVICE_MODULE_TYPE;
    payload[4] = (uint8_t)(module_specific_bits & 0xffU);
    payload[5] = (uint8_t)((module_specific_bits >> 8) & 0xffU);
    payload[6] = (uint8_t)((module_specific_bits >> 16) & 0xffU);
    payload[7] = (uint8_t)((module_specific_bits >> 24) & 0xffU);
    return send_pgn_priority(PGN_CZONE_STATUS, 7, payload, sizeof(payload));
}

static esp_err_t send_czone_oi_status(uint8_t relay_mask)
{
    uint8_t payload[4U + CZONE_OI_STATUS_RECORDS * CZONE_OI_STATUS_RECORD_LEN] = {0};

    write_u16_le(payload, czone_vendor_header());
    payload[2] = CZONE_OI_STATUS_PAGE;
    payload[3] = device_config_get_dipswitch();

    for (uint8_t record = 0; record < CZONE_OI_STATUS_RECORDS; ++record) {
        uint8_t *dst = &payload[4U + record * CZONE_OI_STATUS_RECORD_LEN];
        if ((relay_mask & (uint8_t)(1U << record)) != 0) {
            dst[0] = 0x01;
            dst[1] = 0x01;
            dst[2] = 0x04;
        }
    }

    return send_fast_pgn(PGN_CZONE_OI_STATUS, payload, sizeof(payload));
}

static esp_err_t send_czone_zcf_size_claim(size_t file_size)
{
    ESP_RETURN_ON_FALSE(file_size > 0 && file_size <= 0xfffffU, ESP_ERR_INVALID_SIZE, TAG, "invalid ZCF size");

    uint8_t payload[8] = {0};
    const uint32_t field0 = (uint32_t)file_size & 0xfffffU;

    write_u16_le(payload, czone_vendor_header());
    payload[2] = (uint8_t)(field0 & 0xffU);
    payload[3] = (uint8_t)((field0 >> 8) & 0xffU);
    payload[4] = (uint8_t)((field0 >> 16) & 0x0fU);
    payload[5] = 0;
    payload[6] = 0x80; /* flags=2 tells the CZone tool that ZCF data is ready to receive. */
    /* payload[7] is the CZone module address (the DIP-switch value), which the
     * tool keys its per-module transfer state on -- NOT the N2K source address. */
    payload[7] = device_config_get_dipswitch();

    ESP_LOGI(TAG, "announcing ZCF download size=%u", (unsigned)file_size);
    return send_pgn_priority(PGN_CZONE_CONFIG_CLAIM, 7, payload, sizeof(payload));
}

/* The CZone "config id" a module advertises is the first 3 bytes of the config
 * payload (the bytes after the 6-byte ZCF length/CRC header), as a 20-bit LE
 * value. Returns 0 if no config is stored. */
static uint32_t compute_module_config_id(void)
{
    uint8_t id_bytes[3] = {0};
    size_t got = 0;
    if (zcf_store_read_at(6, id_bytes, sizeof(id_bytes), &got) != ESP_OK || got != sizeof(id_bytes)) {
        return 0;
    }
    return (uint32_t)id_bytes[0] | ((uint32_t)id_bytes[1] << 8) |
           (((uint32_t)id_bytes[2] & 0x0fU) << 16);
}

/* The CZone config "version" (claim field1) is the 20-bit field packed
 * immediately after the config id in the ZCF header: high nibble of byte 8,
 * byte 9, and the low 6 bits of byte 10 -- the same packing as the config id,
 * shifted by four bytes. It is 0 in configs that do not carry a version. */
static uint32_t compute_module_config_version(void)
{
    uint8_t bytes[3] = {0};
    size_t got = 0;
    if (zcf_store_read_at(8, bytes, sizeof(bytes), &got) != ESP_OK || got != sizeof(bytes)) {
        return 0;
    }
    return (uint32_t)(bytes[0] >> 4) | ((uint32_t)bytes[1] << 4) |
           (((uint32_t)bytes[2] & 0x3fU) << 12);
}

/* Advertise this node like a real CZone module so the tool flags us as a
 * write target and adds us to its expected-responders set (dword_4077FA0).
 * Without a registered responder the tool's network-write transmit loop finds
 * sub_583190() vacuously true and skips block 0. Mirrors the broadcast a real
 * module sends: flags=0, field1=0, field0=our stored config id, payload[7]=DIP. */
static esp_err_t send_czone_module_claim(void)
{
    uint8_t payload[8] = {0};
    const uint32_t field0 = compute_module_config_id();

    write_u16_le(payload, czone_vendor_header());
    payload[2] = (uint8_t)(field0 & 0xffU);
    payload[3] = (uint8_t)((field0 >> 8) & 0xffU);
    payload[4] = (uint8_t)((field0 >> 16) & 0x0fU); /* field0 hi nibble; field1 lo nibble = 0 */
    payload[5] = 0;                                  /* field1 mid */
    payload[6] = 0;                                  /* field1 hi (6b) + flags (top 2b) = 0 */
    payload[7] = device_config_get_dipswitch();             /* our CZone module address */

    return send_pgn_priority(PGN_CZONE_CONFIG_CLAIM, 7, payload, sizeof(payload));
}

static esp_err_t send_ss_feedback(const uint8_t *data, uint8_t len)
{
    uint8_t payload[8] = {0};
    write_u16_le(payload, czone_vendor_header());
    payload[2] = len > 2 ? data[2] : 0;
    return send_pgn(PGN_CZONE_SS_FEEDBACK, payload, 3);
}

static esp_err_t send_zcf_block_ack(uint8_t target, uint16_t block_index, uint8_t status)
{
    uint8_t payload[8] = {0xff};
    write_u16_le(&payload[0], czone_vendor_header());
    /* payload[2] must echo the write target (the datablock's flag byte: the
     * target module's DIP, or 0xFF for a broadcast/network write). The tool
     * compares it against its target (byte_40BB158) in sub_5829A0; a mismatch
     * means the ACK is ignored and it re-sends the block forever. */
    payload[2] = target;
    payload[3] = status;
    write_u16_le(&payload[4], block_index);
    /* payload[6] is our CZone module address (DIP switch). The tool indexes its
     * per-block "which modules have acked" bitmap by this value, so it must be
     * the module address, not the N2K source address. */
    payload[6] = device_config_get_dipswitch();
    esp_err_t err = send_pgn(PGN_CZONE_DATABLOCK_ACK, payload, sizeof(payload));
    if (status == 0 && err == ESP_OK) {
    } else {
    }
    return err;
}

static void arm_zcf_send_ack(uint16_t block_index)
{
    s_zcf_send_ack_src = 0xff;
    s_zcf_send_ack_status = 0xff;
    s_zcf_send_ack_block = block_index;
    s_zcf_send_waiting_ack = true;
    if (s_zcf_send_ack_sem) {
        while (xSemaphoreTake(s_zcf_send_ack_sem, 0) == pdTRUE) {
        }
    }
}

static bool wait_zcf_send_ack(uint16_t block_index, uint8_t *status, uint8_t *src)
{
    if (!s_zcf_send_ack_sem) {
        return false;
    }

    const BaseType_t ok = xSemaphoreTake(s_zcf_send_ack_sem, pdMS_TO_TICKS(ZCF_SEND_ACK_TIMEOUT_MS));
    s_zcf_send_waiting_ack = false;
    if (ok != pdTRUE || s_zcf_send_ack_block != block_index) {
        return false;
    }

    if (status) {
        *status = s_zcf_send_ack_status;
    }
    if (src) {
        *src = s_zcf_send_ack_src;
    }
    return true;
}

static esp_err_t send_zcf_payload_block(uint16_t block_index, const uint8_t *chunk, size_t chunk_len)
{
    uint8_t payload[ZCF_DATABLOCK_HEADER_LEN + ZCF_DATABLOCK_CHUNK_MAX] = {0};

    ESP_RETURN_ON_FALSE(chunk_len <= ZCF_DATABLOCK_CHUNK_MAX, ESP_ERR_INVALID_SIZE, TAG, "ZCF block too large");
    if (chunk_len > 0) {
        ESP_RETURN_ON_FALSE(chunk != NULL, ESP_ERR_INVALID_ARG, TAG, "missing ZCF block data");
        memcpy(&payload[ZCF_DATABLOCK_HEADER_LEN], chunk, chunk_len);
    }

    write_u16_le(payload, czone_vendor_header());
    write_u16_le(&payload[2], block_index);
    /* The CZone tool tags each transfer with the module address it parsed from
     * the size-claim's payload[7] (byte_40781C4 -> byte_4077F6A) and drops any
     * datablock whose payload[4] does not match it. Echo the same DIP-switch
     * module address we announced so every block is accepted. */
    payload[4] = device_config_get_dipswitch();

    const uint8_t payload_len = (uint8_t)(ZCF_DATABLOCK_HEADER_LEN + chunk_len);
    return send_fast_pgn(PGN_CZONE_DATABLOCK, payload, payload_len);
}

esp_err_t czone_protocol_send_zcf(void)
{
    if (s_zcf_send_active) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t file_size = 0;
    ESP_RETURN_ON_ERROR(zcf_store_saved_size(&file_size), TAG, "get saved ZCF size");
    ESP_RETURN_ON_FALSE(file_size > 0, ESP_ERR_INVALID_SIZE, TAG, "stored ZCF is empty");
    ESP_RETURN_ON_ERROR(send_czone_zcf_size_claim(file_size), TAG, "announce saved ZCF size");

    s_zcf_send_active = true;
    esp_err_t err = ESP_OK;
    /* The CZone tool only marks reception complete when it receives a
     * zero-length datablock (sub_5827E0: dword_40780B4 == 0 -> state 12), so
     * always append one empty terminator block after the data blocks. */
    const uint16_t data_blocks =
        (uint16_t)((file_size + ZCF_DATABLOCK_CHUNK_MAX - 1) / ZCF_DATABLOCK_CHUNK_MAX);
    const uint16_t total_blocks = data_blocks + 1;
    for (uint16_t block_index = 0; block_index < total_blocks; ++block_index) {
        uint8_t chunk[ZCF_DATABLOCK_CHUNK_MAX] = {0};
        size_t bytes_read = 0;
        const size_t offset = (size_t)block_index * ZCF_DATABLOCK_CHUNK_MAX;
        const size_t remaining = offset < file_size ? file_size - offset : 0;
        const size_t chunk_len = remaining < ZCF_DATABLOCK_CHUNK_MAX ? remaining : ZCF_DATABLOCK_CHUNK_MAX;

        /* The final terminator block carries no data; skip the read so we do
         * not fseek past EOF (which fails on SPIFFS) and abort the transfer. */
        if (chunk_len > 0) {
            err = zcf_store_read_at(offset, chunk, chunk_len, &bytes_read);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "read saved ZCF block failed: %s", esp_err_to_name(err));
                break;
            }
            if (bytes_read != chunk_len) {
                ESP_LOGE(TAG, "short saved ZCF read");
                err = ESP_FAIL;
                break;
            }
        }

        esp_err_t last_err = ESP_ERR_TIMEOUT;
        for (uint8_t attempt = 1; attempt <= ZCF_SEND_MAX_RETRIES; ++attempt) {
            arm_zcf_send_ack(block_index);
            ESP_LOGI(TAG, "sending ZCF block %u len=%u offset=%u/%u attempt=%u",
                     block_index, (unsigned)chunk_len, (unsigned)offset, (unsigned)file_size, attempt);
            err = send_zcf_payload_block(block_index, chunk, chunk_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "send saved ZCF block failed: %s", esp_err_to_name(err));
                last_err = err;
                break;
            }

            uint8_t ack_status = 0xff;
            uint8_t ack_src = 0xff;
            if (wait_zcf_send_ack(block_index, &ack_status, &ack_src)) {
                /* The CZone tool ACKs data blocks with status 0, but the final
                 * zero-length terminator with status 1 (sub_5827E0 sets
                 * dword_40781B4 = 1) to signal reception complete. Accept both. */
                if (ack_status == 0 || (chunk_len == 0 && ack_status == 1)) {
                    ESP_LOGI(TAG, "ZCF block %u ACK from src=%u status=%u", block_index, ack_src, ack_status);
                    last_err = ESP_OK;
                    break;
                }
                ESP_LOGW(TAG, "ZCF block %u rejected by src=%u status=%u", block_index, ack_src, ack_status);
                last_err = ESP_FAIL;
            } else {
                ESP_LOGW(TAG, "ZCF block %u ACK timeout", block_index);
            }
        }
        if (last_err != ESP_OK) {
            ESP_LOGE(TAG, "ZCF block not acknowledged: %s", esp_err_to_name(last_err));
            err = last_err;
            break;
        }
    }

    s_zcf_send_active = false;
    s_zcf_send_waiting_ack = false;
    return err;
}

static esp_err_t handle_zcf_datablock_ack(uint8_t src, const uint8_t *data, uint8_t len)
{
    if (len < 7 || !has_czone_vendor_header(data, len)) {
        return ESP_OK;
    }

    const uint8_t target = data[2];
    const uint8_t status = data[3];
    const uint16_t block_index = read_u16_le(&data[4]);
    const uint8_t ack_source = data[6];

    if (should_log_pgn(PGN_CZONE_DATABLOCK_ACK, data, len)) {
        usb_terminal_printf("[CZone RX] ZCF ACK target=%u status=%u block=%u ack_source=%u src=%u\r\n",
                            target, status, block_index, ack_source, src);
    }

    /* The tool addresses its ACK to the module address (DIP switch) we
     * announced for the download, not our N2K source address. */
    if (target != device_config_get_dipswitch() || !s_zcf_send_waiting_ack || block_index != s_zcf_send_ack_block) {
        return ESP_OK;
    }

    s_zcf_send_ack_src = src;
    s_zcf_send_ack_status = status;
    if (s_zcf_send_ack_sem) {
        xSemaphoreGive(s_zcf_send_ack_sem);
    }

    return ESP_OK;
}

static esp_err_t handle_config_claim(uint8_t src, const uint8_t *data, uint8_t len)
{
    if (src == s_source_address || len < 8 || !has_czone_vendor_header(data, len)) {
        return ESP_OK;
    }

    const uint32_t field0 = (uint32_t)data[2] |
                            ((uint32_t)data[3] << 8) |
                            (((uint32_t)data[4] & 0x0fU) << 16);
    const uint32_t field1 = ((uint32_t)data[4] >> 4) |
                            ((uint32_t)data[5] << 4) |
                            (((uint32_t)data[6] & 0x3fU) << 12);
    const uint8_t flags = data[6] >> 6;
    const uint8_t seq = data[7];

    if (should_log_pgn(PGN_CZONE_CONFIG_CLAIM, data, len)) {
        usb_terminal_printf("[CZone RX] config claim field0=%lu field1=%lu flags=%u seq=0x%02x\r\n",
                            field0, field1, flags, seq);
    }

    /* Arbitration defer: another config holder is claiming (field0 != 0 carries
     * its config id, field1 its version, seq its module address). It outranks us
     * if it advertises a newer version (sub_582360: a non-zero version beats a
     * zero one); on equal version the lower module address wins. */
    if (s_zcf_arb_pending && field0 != 0) {
        const uint32_t my_version = compute_module_config_version();
        const bool peer_outranks = field1 != my_version
                                       ? field1 > my_version
                                       : seq < device_config_get_dipswitch();
        if (peer_outranks) {
            s_zcf_arb_pending = false;
            if (should_log_pgn(PGN_CZONE_CONFIG_CLAIM, data, len)) {
                usb_terminal_printf("[CZone ZCF] deferring send; holder addr=%u ver=%lu outranks ours (ver=%lu)\r\n",
                                    seq, (unsigned long)field1, (unsigned long)my_version);
            }
        }
    }

    /* flags bit1 means the peer is advertising a config transfer. The tool
     * uses field0==0 to request a read ("you send me yours"); a nonzero field0
     * is a write announcement ("I am sending you config id field0"). */
    if ((flags & 0x02U) != 0 && field0 != 0) {
        /* Network-write announce. Reply with our module claim so the tool
         * registers us as a responder and transmits block 0 instead of
         * skipping it. Keep broadcasting it for a few seconds (the tool waits
         * ~2.6 s after announcing before sending the first block). */
        s_module_claim_until = xTaskGetTickCount() + pdMS_TO_TICKS(6000);
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_czone_module_claim());
        if (should_log_pgn(PGN_CZONE_CONFIG_CLAIM, data, len)) {
            usb_terminal_printf("[CZone ZCF] write announce from src=%u; registering as responder\r\n", src);
        }
        return ESP_OK;
    }
    if ((flags & 0x02U) == 0) {
        return ESP_OK;
    }
    /* flags bit1 set and field0 == 0: a read request, fall through to download. */

    if (s_zcf_send_requested || s_zcf_send_active || s_zcf_arb_pending) {
        if (should_log_pgn(PGN_CZONE_CONFIG_CLAIM, data, len)) {
            usb_terminal_printf("[CZone ZCF] download trigger from src=%u ignored; send/arbitration already pending\r\n", src);
        }
        return ESP_OK;
    }

    size_t file_size = 0;
    const esp_err_t saved_err = zcf_store_saved_size(&file_size);
    if (saved_err != ESP_OK || file_size == 0) {
        if (should_log_pgn(PGN_CZONE_CONFIG_CLAIM, data, len)) {
            usb_terminal_printf("[CZone ZCF] download trigger from src=%u ignored; no stored ZCF\r\n", src);
        }
        return ESP_OK;
    }

    /* Do not send immediately: back off proportional to our module address and
     * defer to any lower-address holder that claims first (handled above). The
     * holder with the lowest module address wins and transmits. */
    const uint8_t my_addr = device_config_get_dipswitch();
    const uint32_t backoff_ms = ((uint32_t)(my_addr >> 3) + 20U) * ZCF_ARB_TICK_MS;
    s_zcf_arb_pending = true;
    s_zcf_arb_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(backoff_ms);
    if (s_protocol_task_handle) {
        xTaskNotify(s_protocol_task_handle, PROTOCOL_NOTIFY_ZCF_SEND, eSetBits);
    }

    if (should_log_pgn(PGN_CZONE_CONFIG_CLAIM, data, len)) {
        usb_terminal_printf("[CZone ZCF] read request from src=%u; arbitrating (back-off %lu ms), stored size=%u\r\n",
                            src, (unsigned long)backoff_ms, (unsigned)file_size);
    }
    return ESP_OK;
}

/* Validate the just-received ZCF still in the pending (uncommitted) file:
 *   [0]    marker
 *   [1..4] length field = file_size - 7 (u32 LE)
 *   [5]    CRC-8 over bytes [0..4]
 *   [6..]  payload, with the final byte being CRC-8 over the payload.
 * Prints the computed vs stored values and returns ESP_OK only if all match. */
static esp_err_t validate_pending_zcf_crc(void)
{
    const size_t file_size = zcf_store_bytes_received();
    if (file_size < 7) {
        usb_terminal_printf("[CZone ZCF] CRC check failed: file too short (%u bytes)\r\n",
                            (unsigned)file_size);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t header[6] = {0};
    size_t got = 0;
    ESP_RETURN_ON_ERROR(zcf_store_read_pending_at(0, header, sizeof(header), &got), TAG, "read ZCF header");
    if (got != sizeof(header)) {
        usb_terminal_printf("[CZone ZCF] CRC check failed: short header read\r\n");
        return ESP_FAIL;
    }

    const uint32_t length_field = read_u32_le(&header[1]);
    const uint32_t expected_length = (uint32_t)(file_size - 7);
    const uint8_t header_crc_calc = zcf_crc8_update(0, header, 5);
    const uint8_t header_crc_file = header[5];

    /* Payload CRC over bytes [6 .. file_size-2]; final byte holds the CRC. */
    uint8_t payload_crc_calc = 0;
    size_t remaining = file_size - 7;
    size_t offset = 6;
    while (remaining > 0) {
        uint8_t buf[64];
        const size_t want = remaining < sizeof(buf) ? remaining : sizeof(buf);
        ESP_RETURN_ON_ERROR(zcf_store_read_pending_at(offset, buf, want, &got), TAG, "read ZCF payload");
        if (got != want) {
            usb_terminal_printf("[CZone ZCF] CRC check failed: short payload read at %u\r\n", (unsigned)offset);
            return ESP_FAIL;
        }
        payload_crc_calc = zcf_crc8_update(payload_crc_calc, buf, want);
        offset += want;
        remaining -= want;
    }
    uint8_t payload_crc_file = 0;
    ESP_RETURN_ON_ERROR(zcf_store_read_pending_at(file_size - 1, &payload_crc_file, 1, &got), TAG, "read ZCF payload CRC");

    const bool length_ok = (length_field == expected_length);
    const bool header_ok = (header_crc_calc == header_crc_file);
    const bool payload_ok = (payload_crc_calc == payload_crc_file);

    usb_terminal_printf("[CZone ZCF] CRC check: size=%u length=%lu/%lu hdrCRC=%02x/%02x payloadCRC=%02x/%02x -> %s\r\n",
                        (unsigned)file_size,
                        (unsigned long)length_field, (unsigned long)expected_length,
                        header_crc_calc, header_crc_file,
                        payload_crc_calc, payload_crc_file,
                        (length_ok && header_ok && payload_ok) ? "OK" : "FAIL");

    return (length_ok && header_ok && payload_ok) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

static esp_err_t handle_zcf_datablock(uint8_t src, const uint8_t *payload, uint8_t len)
{
    if (len < ZCF_DATABLOCK_HEADER_LEN || !has_czone_vendor_header(payload, len)) {
        return ESP_OK;
    }

    if (s_zcf_send_requested) {
        s_zcf_send_requested = false;
        if (should_log_pgn(PGN_CZONE_DATABLOCK, payload, len)) {
            usb_terminal_printf("[CZone ZCF] queued download canceled; incoming upload block arrived from src=%u\r\n", src);
        }
    }

    const uint16_t subcode = read_u16_le(&payload[2]);
    const uint8_t block_flag = payload[4];
    const size_t chunk_len = len - ZCF_DATABLOCK_HEADER_LEN;
    const size_t total_before = zcf_store_bytes_received();
    uint8_t chunk[ZCF_DATABLOCK_CHUNK_MAX];
    const bool active = zcf_store_transfer_active();

    if (chunk_len > sizeof(chunk)) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_zcf_block_ack(block_flag, subcode, 1));
        return ESP_ERR_INVALID_SIZE;
    }
    memcpy(chunk, &payload[ZCF_DATABLOCK_HEADER_LEN], chunk_len);
    (void)len;

    if (!active) {
        if (subcode != 0) {
            /* A transfer must begin at block 0. A nonzero subcode here means we
             * have not seen block 0 yet (e.g. it was lost). Storing from a later
             * block would silently truncate the file, so ignore it WITHOUT an
             * ACK -- the tool then retransmits the unacknowledged block 0.
             * Exception: a zero-length terminator left over from an already
             * finished transfer is acked so the tool's handshake can complete. */
            if (chunk_len == 0) {
                size_t saved_size = 0;
                if (zcf_store_saved_size(&saved_size) != ESP_OK) {
                    saved_size = total_before;
                }
                ESP_ERROR_CHECK_WITHOUT_ABORT(send_zcf_block_ack(block_flag, subcode, 0));
            }
            return ESP_OK;
        }
        ESP_RETURN_ON_ERROR(zcf_store_begin_at(0), TAG, "start ZCF upload");
    }
    if (subcode < zcf_store_next_block_index()) {
        /* Already stored; re-ACK so the tool advances past it. */
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_zcf_block_ack(block_flag, subcode, 0));
        return ESP_OK;
    }
    if (subcode > zcf_store_next_block_index()) {
        /* Gap: an earlier block is missing. Ignore without an ACK so the tool
         * retransmits the missing block instead of us storing out of order. */
        return ESP_OK;
    }

    esp_err_t err = zcf_store_append_block(subcode, chunk, chunk_len);
    if (err == ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(send_zcf_block_ack(block_flag, subcode, 0));
        if (chunk_len < ZCF_DATABLOCK_CHUNK_MAX) {
            const esp_err_t crc_err = validate_pending_zcf_crc();
            if (crc_err != ESP_OK) {
                /* Refuse to store a config that fails its internal CRC. */
                usb_terminal_printf("[CZone ZCF] discarding received config: CRC invalid (%s)\r\n",
                                    esp_err_to_name(crc_err));
                ESP_ERROR_CHECK_WITHOUT_ABORT(zcf_store_abort());
                return ESP_OK;
            }
            ESP_ERROR_CHECK_WITHOUT_ABORT(zcf_store_complete());
            usb_terminal_printf("[CZone ZCF] stored config %u bytes (CRC ok)\r\n",
                                (unsigned)zcf_store_bytes_received());

            /* Re-parse the freshly committed config so circuit->relay switching
             * uses the new mapping immediately, without a reboot. */
            const esp_err_t map_err = zcf_config_load();
            usb_terminal_printf("[CZone ZCF] circuit->relay map: %u entr%s for dipswitch 0x%02x (%s)\r\n",
                                (unsigned)zcf_config_mapping_count(),
                                zcf_config_mapping_count() == 1 ? "y" : "ies",
                                device_config_get_dipswitch(), esp_err_to_name(map_err));
        }
        return ESP_OK;
    }

    /* Storage failed (e.g. SPIFFS write error). Do not ACK so the tool
     * retransmits this in-sequence block. */
    return err;
}

static esp_err_t handle_switch_bank_control(const uint8_t *data, uint8_t len)
{
    if (len < 8) {
        return ESP_OK;
    }

    const uint8_t instance = data[0];
    if (instance != 0) {
        if (should_log_pgn(PGN_SWITCH_BANK_CONTROL, data, len)) {
            usb_terminal_printf("[CZone RX] switch bank instance %u ignored; this device exposes bank 0\r\n", instance);
        }
        return ESP_OK;
    }

    uint8_t mask = relay_controller_get_mask();
    for (uint8_t sw = 0; sw < BOARD_RELAY_COUNT; ++sw) {
        const uint8_t value = (data[1 + sw / 4] >> ((sw % 4) * 2)) & 0x03;
        if (value == 0) {
            mask &= (uint8_t)~(1U << sw);
        } else if (value == 1) {
            mask |= (uint8_t)(1U << sw);
        }
    }

    ESP_RETURN_ON_ERROR(relay_controller_set_mask(mask), TAG, "apply switch bank control");
    return czone_protocol_publish_relay_state(mask);
}

static uint8_t temporary_ss_switch_to_relay(uint16_t switch_id)
{
    /*
     * Pre-configuration fallback only.
     *
     * Once a ZCF has been uploaded, circuit->relay switching comes from
     * zcf_config_relays_for_circuit() (parsed from the config for our
     * dipswitch). This hard-coded table is used solely before any ZCF is
     * present; these IDs came from pressing the MFD buttons sequentially during
     * bring-up.
     */
    static const uint16_t switch_ids[] = {
        0x000d, 0x000e, 0x000f, 0x0012, 0x0011, 0x0010,
    };

    for (uint8_t i = 0; i < sizeof(switch_ids) / sizeof(switch_ids[0]); ++i) {
        if (switch_ids[i] == switch_id) {
            return (uint8_t)(i + 1U);
        }
    }
    return 0;
}

static esp_err_t handle_ss_command(uint8_t src, const uint8_t *data, uint8_t len)
{
    if (!has_czone_vendor_header(data, len)) {
        return ESP_OK;
    }

    esp_err_t err = send_ss_feedback(data, len);
    if (len < 8) {
        return err;
    }

    const uint16_t switch_id = read_u16_le(&data[2]);
    const uint8_t target = data[5];
    const uint8_t command = data[6];

    if (target != s_source_address && target != N2K_GLOBAL_ADDRESS) {
        if (should_log_pgn(PGN_CZONE_SS_COMMAND, data, len)) {
            usb_terminal_printf("[CZone RX] SS command switch=0x%04x target=%u ignored; our address=%u\r\n",
                                switch_id, target, s_source_address);
        }
        return err;
    }

    /* Resolve the circuit -> relay output(s) from the parsed ZCF, keeping only
     * outputs addressed to our dipswitch. Fall back to the temporary bring-up
     * table only when no ZCF has been uploaded yet, so the board still responds
     * before it has been configured. */
    uint8_t relays[BOARD_RELAY_COUNT];
    uint8_t relay_count = zcf_config_relays_for_circuit(switch_id, relays, (uint8_t)sizeof(relays));
    const bool from_zcf = relay_count > 0;
    if (relay_count == 0 && !zcf_config_loaded()) {
        const uint8_t fallback = temporary_ss_switch_to_relay(switch_id);
        if (fallback != 0) {
            relays[0] = fallback;
            relay_count = 1;
        }
    }

    if (relay_count == 0) {
        if (should_log_pgn(PGN_CZONE_SS_COMMAND, data, len)) {
            usb_terminal_printf("[CZone RX] SS command switch=0x%04x command=0x%02x maps to no output on dipswitch 0x%02x\r\n",
                                switch_id, command, device_config_get_dipswitch());
        }
        return err;
    }

    if (command == CZONE_SS_CMD_RELEASE) {
        if (should_log_pgn(PGN_CZONE_SS_COMMAND, data, len)) {
            usb_terminal_printf("[CZone RX] SS release switch=0x%04x ignored\r\n", switch_id);
        }
        return err;
    }
    if (command != CZONE_SS_CMD_PRESS && command != CZONE_SS_CMD_PRESS_ALT) {
        if (should_log_pgn(PGN_CZONE_SS_COMMAND, data, len)) {
            usb_terminal_printf("[CZone RX] SS command switch=0x%04x command=0x%02x ignored\r\n",
                                switch_id, command);
        }
        return err;
    }

    for (uint8_t i = 0; i < relay_count; ++i) {
        ESP_RETURN_ON_ERROR(relay_controller_toggle_channel(relays[i]), TAG, "apply SS command");
    }
    const uint8_t mask = relay_controller_get_mask();
    if (should_log_pgn(PGN_CZONE_SS_COMMAND, data, len)) {
        usb_terminal_printf("[CZone RX] SS press switch=0x%04x relays=%u (%s) mask=0x%02x src=%u\r\n",
                            switch_id, relay_count, from_zcf ? "zcf" : "fallback", mask, src);
    }
    ESP_RETURN_ON_ERROR(czone_protocol_publish_relay_state(mask), TAG, "publish SS relay state");
    return err;
}

static void log_reassembled(uint32_t pgn, uint8_t src, const uint8_t *payload, uint8_t len)
{
    if (!should_log_pgn(pgn, payload, len)) {
        return;
    }

    if (pgn == PGN_CZONE_DATABLOCK && len >= ZCF_DATABLOCK_HEADER_LEN) {
        const uint16_t subcode = read_u16_le(&payload[2]);
        const uint8_t flag = payload[4];
        const uint16_t chunk = (uint16_t)(len - ZCF_DATABLOCK_HEADER_LEN);
        log_timestamp();
        usb_terminal_printf("[CZone RX] datablock src=%u block=%u flag=0x%02x data_len=%u%s\r\n",
                            src, subcode, flag, chunk, chunk == 0 ? " (terminator)" : "");
        return;
    }

    log_timestamp();
    usb_terminal_printf("[CZone RX] fast PGN %lu (0x%05lx %s) src=%u payload_len=%u\r\n",
           pgn, pgn, pgn_name(pgn), src, len);

    if (pgn == PGN_CZONE_DEBUG && len > 0) {
        usb_terminal_printf("[CZone Debug] ");
        for (uint8_t i = 0; i < len && payload[i] != 0; ++i) {
            usb_terminal_printf("%c", payload[i] >= 32 && payload[i] <= 126 ? payload[i] : '.');
        }
        usb_terminal_printf("\r\n");
    }
}

esp_err_t czone_protocol_handle_frame(const czone_can_frame_t *frame)
{
    if (!frame || frame->dlc > 8) {
        return ESP_OK;
    }
    if (!frame->extended) {
        if (s_monitor_mode == CZONE_CAN_MONITOR_ALL) {
            usb_terminal_printf("[CAN RX] id=0x%03lx std len=%u data=", frame->can_id, frame->dlc);
            print_data(frame->data, frame->dlc);
            usb_terminal_printf("\r\n");
        }
        return ESP_OK;
    }

    const n2k_id_t id = n2k_decode(frame->can_id);
    log_frame(frame, &id);

    if (is_fast_packet_pgn(id.pgn)) {
        uint8_t payload_len = 0;
        const uint8_t *payload = feed_fast_packet(frame->can_id, id.pgn, id.src, frame->data, frame->dlc, &payload_len);
        if (payload) {
            log_reassembled(id.pgn, id.src, payload, payload_len);
            if (id.pgn == PGN_CZONE_DATABLOCK) {
                return handle_zcf_datablock(id.src, payload, payload_len);
            }
        }
        return ESP_OK;
    }

    switch (id.pgn) {
    case PGN_ISO_ADDRESS_CLAIM:
        handle_address_claim(&id, frame->data, frame->dlc);
        break;

    case PGN_ISO_REQUEST:
        if (frame->dlc >= 3 && (id.dst == s_source_address || id.dst == N2K_GLOBAL_ADDRESS)) {
            const uint32_t requested = (uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8) | ((uint32_t)frame->data[2] << 16);
            if (s_monitor_mode == CZONE_CAN_MONITOR_ALL) {
                usb_terminal_printf("[N2K RX] ISO request for PGN %lu from src=%u\r\n", requested, id.src);
            }
            if (requested == PGN_ISO_ADDRESS_CLAIM) {
                return send_address_claim();
            }
            if (requested == PGN_PGN_LIST) {
                return send_pgn_lists();
            }
            if (requested == PGN_HEARTBEAT) {
                return send_heartbeat();
            }
            if (requested == PGN_PRODUCT_INFO) {
                return send_product_information();
            }
            if (requested == PGN_CONFIG_INFO) {
                return send_config_information();
            }
        }
        break;

    case PGN_SWITCH_BANK_CONTROL:
        return handle_switch_bank_control(frame->data, frame->dlc);

    case PGN_CZONE_SS_COMMAND:
        return handle_ss_command(id.src, frame->data, frame->dlc);

    case PGN_CZONE_DATABLOCK_ACK:
        return handle_zcf_datablock_ack(id.src, frame->data, frame->dlc);

    case PGN_CZONE_CONFIG_CLAIM:
        return handle_config_claim(id.src, frame->data, frame->dlc);

    case PGN_CZONE_STATUS:
        if (!has_czone_vendor_header(frame->data, frame->dlc)) {
            break;
        }
        if (frame->dlc >= 8 && should_log_pgn(id.pgn, frame->data, frame->dlc)) {
            const uint32_t bits = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8) |
                                  ((uint32_t)frame->data[6] << 16) | ((uint32_t)frame->data[7] << 24);
            usb_terminal_printf("[CZone RX] status dipswitch=0x%02x module=0x%02x bits=0x%08lx\r\n",
                                frame->data[2], frame->data[3], bits);
        }
        break;

    case PGN_CZONE_BRIEF_STATUS:
        if (!has_czone_vendor_header(frame->data, frame->dlc)) {
            break;
        }
        if (frame->dlc >= 5 && should_log_pgn(id.pgn, frame->data, frame->dlc)) {
            usb_terminal_printf("[CZone RX] brief config=%u app=%u extra=0x%02x\r\n", frame->data[2], frame->data[3], frame->data[4]);
        }
        break;
    }

    return ESP_OK;
}

esp_err_t czone_protocol_publish_relay_state(uint8_t relay_mask)
{
    ESP_RETURN_ON_ERROR(send_czone_module_status(relay_mask), TAG, "send CZone module status");
    ESP_RETURN_ON_ERROR(send_czone_oi_status(relay_mask), TAG, "send CZone OI status");

    uint8_t brief[8] = {0};
    write_u16_le(brief, czone_vendor_header());
    brief[2] = 5;
    brief[3] = 4;
    brief[4] = 0;
    brief[5] = 0xff;
    brief[6] = 0xff;
    brief[7] = 0xff;
    ESP_RETURN_ON_ERROR(send_pgn(PGN_CZONE_BRIEF_STATUS, brief, sizeof(brief)), TAG, "send CZone brief status");

    ESP_RETURN_ON_ERROR(send_switch_bank_status(relay_mask), TAG, "send switch bank status");

    return ESP_OK;
}
