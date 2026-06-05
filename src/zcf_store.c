// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Grigory Morozov

#include "zcf_store.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_spiffs.h"

static const char *TAG = "zcf_store";

#define ZCF_BASE_PATH "/spiffs"
#define ZCF_TMP_PATH  ZCF_BASE_PATH "/config.zcf.tmp"
#define ZCF_PATH      ZCF_BASE_PATH "/config.zcf"

static bool s_mounted;
static bool s_active;
static size_t s_bytes_received;
static uint16_t s_next_block;

esp_err_t zcf_store_init(void)
{
    if (s_mounted) {
        return ESP_OK;
    }

    const esp_vfs_spiffs_conf_t conf = {
        .base_path = ZCF_BASE_PATH,
        .partition_label = "storage",
        .max_files = 3,
        .format_if_mount_failed = true,
    };

    ESP_RETURN_ON_ERROR(esp_vfs_spiffs_register(&conf), TAG, "mount SPIFFS");
    s_mounted = true;

    size_t total = 0;
    size_t used = 0;
    esp_err_t err = esp_spiffs_info("storage", &total, &used);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    }
    return ESP_OK;
}

esp_err_t zcf_store_begin(void)
{
    return zcf_store_begin_at(0);
}

esp_err_t zcf_store_begin_at(uint16_t first_block_index)
{
    ESP_RETURN_ON_ERROR(zcf_store_init(), TAG, "init ZCF store");

    remove(ZCF_TMP_PATH);
    FILE *file = fopen(ZCF_TMP_PATH, "wb");
    ESP_RETURN_ON_FALSE(file != NULL, ESP_FAIL, TAG, "open temp ZCF for write");
    fclose(file);

    s_active = true;
    s_bytes_received = 0;
    s_next_block = first_block_index;
    ESP_LOGI(TAG, "started ZCF receive at block %u", first_block_index);
    return ESP_OK;
}

esp_err_t zcf_store_append_block(uint16_t block_index, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(data || len == 0, ESP_ERR_INVALID_ARG, TAG, "invalid ZCF block");
    if (!s_active) {
        ESP_RETURN_ON_ERROR(zcf_store_begin(), TAG, "begin implicit ZCF receive");
    }
    ESP_RETURN_ON_FALSE(block_index == s_next_block, ESP_ERR_INVALID_STATE, TAG,
                        "unexpected ZCF block %u, expected %u", block_index, s_next_block);

    FILE *file = fopen(ZCF_TMP_PATH, "ab");
    ESP_RETURN_ON_FALSE(file != NULL, ESP_FAIL, TAG, "open temp ZCF for append");

    const size_t written = len == 0 ? 0 : fwrite(data, 1, len, file);
    const int close_rc = fclose(file);
    ESP_RETURN_ON_FALSE(written == len && close_rc == 0, ESP_FAIL, TAG, "write temp ZCF block");

    s_bytes_received += len;
    ++s_next_block;
    ESP_LOGI(TAG, "stored ZCF block %u len=%u total=%u", block_index, (unsigned)len, (unsigned)s_bytes_received);
    return ESP_OK;
}

esp_err_t zcf_store_complete(void)
{
    ESP_RETURN_ON_FALSE(s_active, ESP_ERR_INVALID_STATE, TAG, "no active ZCF receive");

    remove(ZCF_PATH);
    ESP_RETURN_ON_FALSE(rename(ZCF_TMP_PATH, ZCF_PATH) == 0, ESP_FAIL, TAG, "commit ZCF file");
    s_active = false;
    ESP_LOGI(TAG, "ZCF receive complete: %u bytes stored at %s", (unsigned)s_bytes_received, ZCF_PATH);
    return ESP_OK;
}

esp_err_t zcf_store_abort(void)
{
    s_active = false;
    s_bytes_received = 0;
    s_next_block = 0;
    remove(ZCF_TMP_PATH);
    ESP_LOGW(TAG, "ZCF receive aborted; pending file discarded");
    return ESP_OK;
}

esp_err_t zcf_store_saved_size(size_t *size)
{
    ESP_RETURN_ON_FALSE(size != NULL, ESP_ERR_INVALID_ARG, TAG, "missing size output");
    ESP_RETURN_ON_ERROR(zcf_store_init(), TAG, "init ZCF store");

    struct stat st;
    if (stat(ZCF_PATH, &st) != 0) {
        *size = 0;
        return ESP_ERR_NOT_FOUND;
    }

    *size = (size_t)st.st_size;
    return ESP_OK;
}

esp_err_t zcf_store_read_at(size_t offset, uint8_t *data, size_t len, size_t *bytes_read)
{
    ESP_RETURN_ON_FALSE(data != NULL || len == 0, ESP_ERR_INVALID_ARG, TAG, "missing read buffer");
    ESP_RETURN_ON_FALSE(bytes_read != NULL, ESP_ERR_INVALID_ARG, TAG, "missing read count output");
    ESP_RETURN_ON_ERROR(zcf_store_init(), TAG, "init ZCF store");

    *bytes_read = 0;
    FILE *file = fopen(ZCF_PATH, "rb");
    ESP_RETURN_ON_FALSE(file != NULL, ESP_ERR_NOT_FOUND, TAG, "open saved ZCF for read");

    esp_err_t err = ESP_OK;
    if (fseek(file, (long)offset, SEEK_SET) != 0) {
        err = ESP_FAIL;
    } else if (len > 0) {
        *bytes_read = fread(data, 1, len, file);
        if (ferror(file)) {
            err = ESP_FAIL;
        }
    }

    const int close_rc = fclose(file);
    ESP_RETURN_ON_FALSE(close_rc == 0, ESP_FAIL, TAG, "close saved ZCF");
    return err;
}

esp_err_t zcf_store_read_pending_at(size_t offset, uint8_t *data, size_t len, size_t *bytes_read)
{
    ESP_RETURN_ON_FALSE(data != NULL || len == 0, ESP_ERR_INVALID_ARG, TAG, "missing read buffer");
    ESP_RETURN_ON_FALSE(bytes_read != NULL, ESP_ERR_INVALID_ARG, TAG, "missing read count output");
    ESP_RETURN_ON_ERROR(zcf_store_init(), TAG, "init ZCF store");

    *bytes_read = 0;
    FILE *file = fopen(ZCF_TMP_PATH, "rb");
    ESP_RETURN_ON_FALSE(file != NULL, ESP_ERR_NOT_FOUND, TAG, "open pending ZCF for read");

    esp_err_t err = ESP_OK;
    if (fseek(file, (long)offset, SEEK_SET) != 0) {
        err = ESP_FAIL;
    } else if (len > 0) {
        *bytes_read = fread(data, 1, len, file);
        if (ferror(file)) {
            err = ESP_FAIL;
        }
    }

    const int close_rc = fclose(file);
    ESP_RETURN_ON_FALSE(close_rc == 0, ESP_FAIL, TAG, "close pending ZCF");
    return err;
}

size_t zcf_store_bytes_received(void)
{
    return s_bytes_received;
}

uint16_t zcf_store_next_block_index(void)
{
    return s_next_block;
}

bool zcf_store_transfer_active(void)
{
    return s_active;
}
