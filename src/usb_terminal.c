// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "usb_terminal.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "czone_can.h"
#include "czone_device_identity.h"
#include "czone_protocol.h"
#include "device_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "relay_controller.h"
#include "zcf_config.h"
#include "zcf_store.h"

static const char *TAG = "usb_terminal";
static bool s_usb_ready;

static int usb_write(const char *data, size_t len)
{
    if (!s_usb_ready || len == 0) {
        return 0;
    }

    return usb_serial_jtag_write_bytes(data, len, 0);
}

int usb_terminal_printf(const char *fmt, ...)
{
    char buffer[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, ap);
    va_end(ap);

    if (len <= 0) {
        return len;
    }
    if ((size_t)len >= sizeof(buffer)) {
        len = (int)sizeof(buffer) - 1;
    }

    return usb_write(buffer, (size_t)len);
}

static int usb_log_vprintf(const char *fmt, va_list ap)
{
    char buffer[256];
    int len = vsnprintf(buffer, sizeof(buffer), fmt, ap);

    if (len <= 0) {
        return len;
    }
    if ((size_t)len >= sizeof(buffer)) {
        len = (int)sizeof(buffer) - 1;
    }

    int written = 0;
    for (int i = 0; i < len; ++i) {
        if (buffer[i] == '\n' && (i == 0 || buffer[i - 1] != '\r')) {
            written += usb_write("\r", 1);
        }
        written += usb_write(&buffer[i], 1);
    }
    return written;
}

static char *skip_space(char *text)
{
    while (*text && isspace((unsigned char)*text)) {
        ++text;
    }
    return text;
}

static void trim_right(char *text)
{
    size_t len = strlen(text);
    while (len > 0 && isspace((unsigned char)text[len - 1])) {
        text[--len] = '\0';
    }
}

static void to_lower_ascii(char *text)
{
    for (; *text; ++text) {
        *text = (char)tolower((unsigned char)*text);
    }
}

static bool parse_channel(const char *text, uint8_t *channel)
{
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 10);

    if (end == text || *skip_space(end) != '\0' || value < 1 || value > BOARD_RELAY_COUNT) {
        return false;
    }

    *channel = (uint8_t)value;
    return true;
}

/* Accept an 8-bit binary string (e.g. "00011000"), or a 0x.. / decimal value. */
static bool parse_dipswitch(const char *text, uint8_t *value)
{
    const size_t len = strlen(text);
    bool binary = (len == 8);
    for (size_t i = 0; i < len && binary; ++i) {
        if (text[i] != '0' && text[i] != '1') {
            binary = false;
        }
    }

    if (binary) {
        uint8_t v = 0;
        for (size_t i = 0; i < 8; ++i) {
            v = (uint8_t)((v << 1) | (uint8_t)(text[i] - '0'));
        }
        *value = v;
        return true;
    }

    char *end = NULL;
    const unsigned long parsed = strtoul(text, &end, 0);
    if (end == text || *skip_space(end) != '\0' || parsed > 0xff) {
        return false;
    }
    *value = (uint8_t)parsed;
    return true;
}

static bool parse_mask(const char *text, uint8_t *mask)
{
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 0);

    if (end == text || *skip_space(end) != '\0' || value > 0xff) {
        return false;
    }

    *mask = (uint8_t)value;
    return true;
}

static void print_state(void)
{
    const uint8_t mask = relay_controller_get_mask();
    czone_can_stats_t can_stats = {0};
    usb_terminal_printf("relay mask: 0x%02x\r\n", mask);
    for (uint8_t channel = 1; channel <= BOARD_RELAY_COUNT; ++channel) {
        const bool enabled = (mask & (uint8_t)(1U << (channel - 1U))) != 0;
        usb_terminal_printf("  ch%u: %s\r\n", channel, enabled ? "on" : "off");
    }
    czone_can_get_stats(&can_stats);
    usb_terminal_printf("CAN RX: hw_full=%lu app_full=%lu enqueued=%lu processed=%lu\r\n",
                        can_stats.twai_rx_queue_full, can_stats.app_rx_queue_full,
                        can_stats.app_rx_enqueued, can_stats.app_rx_processed);
    usb_terminal_printf("CAN HW: missed=%lu overrun=%lu bus_err=%lu arb_lost=%lu tec=%lu rec=%lu\r\n",
                        can_stats.hw_rx_missed_count, can_stats.hw_rx_overrun_count,
                        can_stats.hw_bus_error_count, can_stats.hw_arb_lost_count,
                        can_stats.hw_tx_error_counter, can_stats.hw_rx_error_counter);
}

static void print_zcf_status(void)
{
    size_t saved_size = 0;
    const esp_err_t saved_err = zcf_store_saved_size(&saved_size);

    if (saved_err == ESP_OK) {
        usb_terminal_printf("ZCF file: present (%u bytes)\r\n", (unsigned)saved_size);
    } else if (saved_err == ESP_ERR_NOT_FOUND) {
        usb_terminal_printf("ZCF file: not present\r\n");
    } else {
        usb_terminal_printf("ZCF file: status error 0x%x\r\n", saved_err);
    }

    if (zcf_store_transfer_active()) {
        usb_terminal_printf("ZCF receive: active, next block %u, received %u bytes\r\n",
                            zcf_store_next_block_index(), (unsigned)zcf_store_bytes_received());
    } else {
        usb_terminal_printf("ZCF receive: idle\r\n");
    }

    if (zcf_config_loaded()) {
        usb_terminal_printf("ZCF config: loaded, %u relay mapping(s) for dipswitch 0x%02x\r\n",
                            (unsigned)zcf_config_mapping_count(), device_config_get_dipswitch());
    } else {
        usb_terminal_printf("ZCF config: not loaded\r\n");
    }
}

static void print_circuit_map(void)
{
    if (!zcf_config_loaded()) {
        usb_terminal_printf("ZCF circuit map: none loaded (upload a config or check 'zcf')\r\n");
        return;
    }

    const size_t count = zcf_config_mapping_count();
    usb_terminal_printf("ZCF circuit -> output map for dipswitch 0x%02x (%u entr%s):\r\n",
                        device_config_get_dipswitch(), (unsigned)count, count == 1 ? "y" : "ies");
    if (count == 0) {
        usb_terminal_printf("  (this config assigns no outputs to our dipswitch)\r\n");
        return;
    }

    const uint8_t mask = relay_controller_get_mask();
    for (size_t i = 0; i < count; ++i) {
        uint8_t circuit = 0;
        uint8_t relay = 0;
        uint16_t level = 0;
        const char *circuit_name = NULL;
        const char *load_name = NULL;
        if (zcf_config_get_entry(i, &circuit, &relay, &level, &circuit_name, &load_name)) {
            const bool on = (mask & (uint8_t)(1U << (relay - 1U))) != 0;
            usb_terminal_printf("  circuit %3u '%s' -> relay %u '%s'  level %-4u  [%s]\r\n",
                                circuit, circuit_name ? circuit_name : "",
                                relay, load_name ? load_name : "",
                                level, on ? "on" : "off");
        }
    }
}

static void print_help(void)
{
    usb_terminal_printf("\r\nCommands:\r\n");
    usb_terminal_printf("  help                 Show this help\r\n");
    usb_terminal_printf("  reboot               Restart the device\r\n");
    usb_terminal_printf("  status               Show relay states\r\n");
    usb_terminal_printf("  on <1-8>             Turn one relay on\r\n");
    usb_terminal_printf("  off <1-8>            Turn one relay off\r\n");
    usb_terminal_printf("  toggle <1-8>         Toggle one relay\r\n");
    usb_terminal_printf("  mask <0x00-0xff>     Set all relay outputs by bit mask\r\n");
    usb_terminal_printf("  all on               Turn all relays on\r\n");
    usb_terminal_printf("  all off              Turn all relays off\r\n\r\n");
    usb_terminal_printf("  zcf                  Show stored ZCF file status\r\n");
    usb_terminal_printf("  czone map            Show Circuit ID -> relay output mapping (our dipswitch)\r\n");
    usb_terminal_printf("  czone dipswitch <b>  Set module dipswitch from 8-bit binary, e.g. 00011000 (saved to NVS)\r\n\r\n");
    usb_terminal_printf("  canmonitor all       Log every received CAN frame\r\n");
    usb_terminal_printf("  canmonitor czone     Log received CZone PGNs only\r\n");
    usb_terminal_printf("  canmonitor zcf       Log ZCF transfer PGNs only (claim/datablock/ack)\r\n");
    usb_terminal_printf("  canmonitor off       Disable CAN receive logging\r\n");
    usb_terminal_printf("  Esc                  Disable CAN receive logging\r\n\r\n");
}

static esp_err_t publish_state(void)
{
    return czone_can_publish_relay_state(relay_controller_get_mask());
}

static esp_err_t handle_command(char *line)
{
    trim_right(line);
    char *command = skip_space(line);
    to_lower_ascii(command);

    if (*command == '\0') {
        return ESP_OK;
    }

    char *arg = command;
    while (*arg && !isspace((unsigned char)*arg)) {
        ++arg;
    }
    if (*arg) {
        *arg++ = '\0';
    }
    arg = skip_space(arg);

    if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
        print_help();
        return ESP_OK;
    }

    if (strcmp(command, "reboot") == 0 || strcmp(command, "restart") == 0) {
        usb_terminal_printf("Rebooting...\r\n");
        vTaskDelay(pdMS_TO_TICKS(100)); /* let the message flush over USB */
        esp_restart();
        return ESP_OK; /* not reached */
    }

    if (strcmp(command, "status") == 0) {
        print_state();
        usb_terminal_printf("CAN monitor: %s\r\n", czone_protocol_monitor_mode_name(czone_protocol_get_monitor_mode()));
        return ESP_OK;
    }

    if (strcmp(command, "zcf") == 0) {
        print_zcf_status();
        return ESP_OK;
    }

    if (strcmp(command, "czone") == 0) {
        /* Split the argument into a sub-command and its value. */
        char *sub = arg;
        char *value = sub;
        while (*value && !isspace((unsigned char)*value)) {
            ++value;
        }
        if (*value) {
            *value++ = '\0';
        }
        value = skip_space(value);

        if (strcmp(sub, "map") == 0 || *sub == '\0') {
            print_circuit_map();
            return ESP_OK;
        }

        if (strcmp(sub, "dipswitch") == 0) {
            if (*value == '\0') {
                const uint8_t cur = device_config_get_dipswitch();
                usb_terminal_printf("dipswitch: 0x%02x (%u, binary ", cur, cur);
                for (int b = 7; b >= 0; --b) {
                    usb_terminal_printf("%d", (cur >> b) & 1);
                }
                usb_terminal_printf(")\r\n");
                return ESP_OK;
            }

            uint8_t dip = 0;
            ESP_RETURN_ON_FALSE(parse_dipswitch(value, &dip), ESP_ERR_INVALID_ARG, TAG,
                                "expected 8-bit binary, e.g. 'czone dipswitch 00011000'");
            ESP_RETURN_ON_ERROR(device_config_set_dipswitch(dip), TAG, "persist dipswitch");

            /* Re-parse the stored ZCF so the circuit->relay map reflects the new
             * module address immediately. */
            (void)zcf_config_load();

            usb_terminal_printf("dipswitch set to 0x%02x (binary ", dip);
            for (int b = 7; b >= 0; --b) {
                usb_terminal_printf("%d", (dip >> b) & 1);
            }
            usb_terminal_printf("), saved to NVS\r\n");
            print_circuit_map();
            return ESP_OK;
        }

        ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_ARG, TAG,
                            "expected 'czone map' or 'czone dipswitch <8-bit binary>'");
    }

    if (strcmp(command, "canmonitor") == 0) {
        czone_can_monitor_mode_t mode = CZONE_CAN_MONITOR_OFF;
        if (strcmp(arg, "all") == 0) {
            mode = CZONE_CAN_MONITOR_ALL;
        } else if (strcmp(arg, "czone") == 0) {
            mode = CZONE_CAN_MONITOR_CZONE;
        } else if (strcmp(arg, "zcf") == 0) {
            mode = CZONE_CAN_MONITOR_ZCF;
        } else if (strcmp(arg, "off") == 0) {
            mode = CZONE_CAN_MONITOR_OFF;
        } else {
            ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_ARG, TAG, "expected 'canmonitor all', 'canmonitor czone', 'canmonitor zcf', or 'canmonitor off'");
        }

        czone_protocol_set_monitor_mode(mode);
        usb_terminal_printf("CAN monitor: %s\r\n", czone_protocol_monitor_mode_name(mode));
        return ESP_OK;
    }

    if (strcmp(command, "on") == 0 || strcmp(command, "off") == 0 || strcmp(command, "toggle") == 0) {
        uint8_t channel = 0;
        ESP_RETURN_ON_FALSE(parse_channel(arg, &channel), ESP_ERR_INVALID_ARG, TAG, "expected channel 1-8");

        if (strcmp(command, "toggle") == 0) {
            ESP_RETURN_ON_ERROR(relay_controller_toggle_channel(channel), TAG, "toggle relay");
        } else {
            ESP_RETURN_ON_ERROR(relay_controller_set_channel(channel, strcmp(command, "on") == 0), TAG, "set relay");
        }

        ESP_RETURN_ON_ERROR(publish_state(), TAG, "publish relay state");
        print_state();
        return ESP_OK;
    }

    if (strcmp(command, "mask") == 0) {
        uint8_t mask = 0;
        ESP_RETURN_ON_FALSE(parse_mask(arg, &mask), ESP_ERR_INVALID_ARG, TAG, "expected mask 0x00-0xff");
        ESP_RETURN_ON_ERROR(relay_controller_set_mask(mask), TAG, "set relay mask");
        ESP_RETURN_ON_ERROR(publish_state(), TAG, "publish relay state");
        print_state();
        return ESP_OK;
    }

    if (strcmp(command, "all") == 0) {
        uint8_t mask = 0;
        if (strcmp(arg, "on") == 0) {
            mask = 0xff;
        } else if (strcmp(arg, "off") == 0) {
            mask = 0x00;
        } else {
            ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_ARG, TAG, "expected 'all on' or 'all off'");
        }

        ESP_RETURN_ON_ERROR(relay_controller_set_mask(mask), TAG, "set all relays");
        ESP_RETURN_ON_ERROR(publish_state(), TAG, "publish relay state");
        print_state();
        return ESP_OK;
    }

    ESP_LOGW(TAG, "unknown command: %s", command);
    print_help();
    return ESP_ERR_NOT_FOUND;
}

static void terminal_task(void *arg)
{
    (void)arg;
    char line[96];
    size_t line_len = 0;

    esp_log_set_vprintf(usb_log_vprintf);
    usb_terminal_printf("\r\n%s USB relay terminal ready. Type 'help'.\r\n> ", CZONE_DEVICE_NAME);

    while (true) {
        uint8_t rx[32];
        const int rx_size = usb_serial_jtag_read_bytes(rx, sizeof(rx), pdMS_TO_TICKS(20));
        if (rx_size <= 0) {
            continue;
        }

        for (int i = 0; i < rx_size; ++i) {
            uint8_t byte = rx[i];

            if (byte == 0x1b) {
                czone_protocol_set_monitor_mode(CZONE_CAN_MONITOR_OFF);
                line_len = 0;
                usb_terminal_printf("\r\nCAN monitor: off\r\n> ");
                continue;
            }

            if (byte == '\r' || byte == '\n') {
                if (line_len == 0) {
                    usb_terminal_printf("\r\n> ");
                    continue;
                }

                line[line_len] = '\0';
                usb_terminal_printf("\r\n");
                const esp_err_t err = handle_command(line);
                if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                    usb_terminal_printf("error: %s\r\n", esp_err_to_name(err));
                }
                line_len = 0;
                usb_terminal_printf("> ");
                continue;
            }

            if (byte == 0x08 || byte == 0x7f) {
                if (line_len > 0) {
                    --line_len;
                    usb_terminal_printf("\b \b");
                }
                continue;
            }

            if (line_len + 1 < sizeof(line) && byte >= 32 && byte <= 126) {
                line[line_len++] = (char)byte;
                usb_write((const char *)&byte, 1);
            }
        }
    }
}

esp_err_t usb_terminal_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 2048,
        .rx_buffer_size = 256,
    };
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&cfg), TAG, "install USB Serial/JTAG");
    s_usb_ready = true;

    BaseType_t ok = xTaskCreate(terminal_task, "usb_terminal", 6144, NULL, 10, NULL);
    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_ERR_NO_MEM, TAG, "create USB terminal task");
    return ESP_OK;
}
