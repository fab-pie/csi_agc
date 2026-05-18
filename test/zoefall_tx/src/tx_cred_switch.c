/*
 * tx_cred_switch.c
 *
 * Staged upstream WiFi credential switch for the TX firmware.
 *
 * When a remote config update writes up_ssid_new / up_pass_new (and optionally
 * up_activation_ts) to NVS, this module picks them up at boot and safely tests
 * the new credentials before committing them:
 *
 *   1. If up_activation_ts == 0: attempt switch immediately.
 *      If up_activation_ts >  0: wait until that Unix timestamp, then attempt.
 *   2. Disconnect STA, reconnect with new credentials, wait for IP (30 s timeout).
 *   3. Success: commit up_ssid_new → up_ssid, up_pass_new → up_pass in NVS+backup,
 *               erase staged keys, send {"cred_switch":"success"} to TB.
 *      Failure: reconnect with old credentials,
 *               erase staged keys, send {"cred_switch":"failed"} to TB.
 */

// -------------------------------------------------------------------
#pragma region INCLUDES
// -------------------------------------------------------------------

#define TAG "ZF_TX_CRED"
#define FILE_ID 0x25
#include "zoecare/logs/logs.h"

#include "zoecare/tx/tx_cred_switch.h"
#include "zoecare/tx/config.h"
#include "zoecare/report/report.h"
#include "zc_debug.h"

#include <string.h>
#include <time.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// -------------------------------------------------------------------
#pragma region CONSTANTS
// -------------------------------------------------------------------

#define CRED_SWITCH_CONNECT_TIMEOUT_MS  30000
#define WIFI_CONNECTED_BIT              BIT0

// -------------------------------------------------------------------
#pragma region STATIC STATE
// -------------------------------------------------------------------

static EventGroupHandle_t s_wifi_eg = NULL;

// -------------------------------------------------------------------
#pragma region NVS HELPERS
// -------------------------------------------------------------------

#define NVS_NAMESPACE "zoefall"

static esp_err_t _sync_to_backup_local(const char *key, const void *value, nvs_type_t type)
{
    const esp_partition_t *bkp =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x41, "nvs_backup");
    if (!bkp) return ESP_ERR_NOT_FOUND;

    esp_err_t err = nvs_flash_init_partition_ptr(bkp);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        return err;
    if (err != ESP_OK) return err;

    nvs_handle_t h;
    err = nvs_open_from_partition(bkp->label, NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) { nvs_flash_deinit_partition(bkp->label); return err; }

    switch (type) {
        case NVS_TYPE_STR: err = nvs_set_str(h, key, (const char *)value); break;
        default:           err = ESP_ERR_INVALID_ARG; break;
    }
    if (err == ESP_OK) nvs_commit(h);
    nvs_close(h);
    nvs_flash_deinit_partition(bkp->label);
    return err;
}

/* Write a string key to main NVS and mirror to backup. */
static void _nv_set_str(nvs_handle_t h, const char *key, const char *value)
{
    esp_err_t e = nvs_set_str(h, key, value);
    if (e == ESP_OK)
        _sync_to_backup_local(key, value, NVS_TYPE_STR);
    else
        LOGW("cred_switch: nvs_set_str('%s') failed: %s", key, esp_err_to_name(e));
}

/* Erase a key from main NVS and backup (ignore NOT_FOUND). */
static void _nv_erase(nvs_handle_t h, const char *key)
{
    nvs_erase_key(h, key);

    const esp_partition_t *bkp =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x41, "nvs_backup");
    if (!bkp) return;
    esp_err_t err = nvs_flash_init_partition_ptr(bkp);
    if (err != ESP_OK && err != ESP_ERR_NVS_NO_FREE_PAGES &&
        err != ESP_ERR_NVS_NEW_VERSION_FOUND) return;
    nvs_handle_t bh;
    if (nvs_open_from_partition(bkp->label, NVS_NAMESPACE, NVS_READWRITE, &bh) == ESP_OK) {
        nvs_erase_key(bh, key);
        nvs_commit(bh);
        nvs_close(bh);
    }
    nvs_flash_deinit_partition(bkp->label);
}

// -------------------------------------------------------------------
#pragma region WIFI HELPERS
// -------------------------------------------------------------------

/* Reconfigure STA with given SSID/password and reconnect. */
static void _sta_reconnect(const char *ssid, const char *password)
{
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid,     ssid,     sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, password, sizeof(cfg.sta.password));
    esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg);
    esp_wifi_connect();
}

// -------------------------------------------------------------------
#pragma region SWITCH TASK
// -------------------------------------------------------------------

static void tx_cred_switch_task(void *arg)
{
    /* --- Phase 1: wait for activation timestamp (if any) --- */
    uint32_t activation_ts = zc_zf_tx_cfg.upstream_activation_ts;
    if (activation_ts > 0) {
        time_t now = time(NULL);
        if ((uint32_t)now < activation_ts) {
            uint32_t wait_s = activation_ts - (uint32_t)now;
            LOGI("cred_switch: scheduled switch in %u s", (unsigned)wait_s);
            /* Chunked sleep so we don't block forever if time jumps */
            while ((uint32_t)time(NULL) < activation_ts) {
                vTaskDelay(pdMS_TO_TICKS(10000));
            }
        }
    }

    /* --- Phase 2: save current credentials and attempt switch --- */
    char old_ssid[64]     = {0};
    char old_password[64] = {0};
    strlcpy(old_ssid,     zc_zf_tx_cfg.upstream_ssid,     sizeof(old_ssid));
    strlcpy(old_password, zc_zf_tx_cfg.upstream_password, sizeof(old_password));

    const char *new_ssid = zc_zf_tx_cfg.upstream_ssid_new;
    const char *new_pass = zc_zf_tx_cfg.upstream_password_new;

    ZC_LOG_SENS(TAG, "cred_switch: testing new upstream credentials (SSID: %s)", new_ssid);

    /* Clear the connected bit so we can wait fresh */
    if (s_wifi_eg) xEventGroupClearBits(s_wifi_eg, WIFI_CONNECTED_BIT);

    _sta_reconnect(new_ssid, new_pass);

    /* Wait for IP acquisition */
    EventBits_t bits = 0;
    if (s_wifi_eg) {
        bits = xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                                   pdFALSE, pdTRUE,
                                   pdMS_TO_TICKS(CRED_SWITCH_CONNECT_TIMEOUT_MS));
    }

    bool success = (bits & WIFI_CONNECTED_BIT) != 0;

    /* Open main NVS for writes */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        LOGE("cred_switch: nvs_open failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    if (success) {
        /* --- Commit new credentials --- */
        LOGI("cred_switch: IP acquired, committing new credentials");
        _nv_set_str(h, "up_ssid", new_ssid);
        _nv_set_str(h, "up_pass", new_pass);
        /* Update in-memory struct so subsequent operations use the right creds */
        strlcpy(zc_zf_tx_cfg.upstream_ssid,     new_ssid, sizeof(zc_zf_tx_cfg.upstream_ssid));
        strlcpy(zc_zf_tx_cfg.upstream_password, new_pass, sizeof(zc_zf_tx_cfg.upstream_password));
    } else {
        /* --- Revert to old credentials --- */
        LOGW("cred_switch: timeout, reverting to old credentials");
        _sta_reconnect(old_ssid, old_password);
    }

    /* Erase staged keys regardless of outcome */
    _nv_erase(h, "up_ssid_new");
    _nv_erase(h, "up_pass_new");
    _nv_erase(h, "up_activation_ts");
    nvs_commit(h);
    nvs_close(h);

    /* Clear staged fields in the in-memory struct */
    memset(zc_zf_tx_cfg.upstream_ssid_new,      0, sizeof(zc_zf_tx_cfg.upstream_ssid_new));
    memset(zc_zf_tx_cfg.upstream_password_new,  0, sizeof(zc_zf_tx_cfg.upstream_password_new));
    zc_zf_tx_cfg.upstream_activation_ts = 0;

    /* Send telemetry */
    const char *result = success ? "success" : "failed";
    char tb_body[48];
    snprintf(tb_body, sizeof(tb_body), "{\"cred_switch\":\"%s\"}", result);
    zc_report_send(tb_body, strlen(tb_body));

    LOGI("cred_switch: done, result=%s", result);
    vTaskDelete(NULL);
}

// -------------------------------------------------------------------
#pragma region PUBLIC API
// -------------------------------------------------------------------

void tx_cred_switch_init(EventGroupHandle_t wifi_eg)
{
    s_wifi_eg = wifi_eg;

    /* Only proceed if staged credentials are present in NVS */
    if (zc_zf_tx_cfg.upstream_ssid_new[0] == '\0') {
        return;
    }

    ZC_LOG_SENS(TAG, "cred_switch: staged credentials found (SSID: %s, activation_ts: %u)",
         zc_zf_tx_cfg.upstream_ssid_new,
         (unsigned)zc_zf_tx_cfg.upstream_activation_ts);

    xTaskCreate(tx_cred_switch_task, "tx_cred_sw", 4096, NULL, 5, NULL);
}
