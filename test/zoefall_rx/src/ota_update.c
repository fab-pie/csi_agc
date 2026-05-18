/*
    logs
*/
#define TAG "ZF_RX_OTA"
#define FILE_ID 0x1B
#include "zoecare/logs/logs.h"

#include "zoecare/rx/ota_update.h"
#include "zoecare/rx/config.h"
#include "zoecare/report/report.h"
#include "zoecare/report/maintenance.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "esp_ota_ops.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"


volatile bool ota_in_progress = false;

// Send OTA status telemetry to ThingsBoard.
// status:  "downloading" | "success" | "rollback"
// version: target or running version string (may be NULL)
// error:   error description for rollback (may be NULL)
static void _report_ota_status(const char *status, const char *version, const char *error)
{
    char body[256];
    if (error && error[0]) {
        snprintf(body, sizeof(body),
                 "{\"ota_status\":\"%s\",\"firmware_version\":\"%s\",\"ota_error\":\"%s\"}",
                 status, version ? version : "", error);
    } else if (version && version[0]) {
        snprintf(body, sizeof(body),
                 "{\"ota_status\":\"%s\",\"firmware_version\":\"%s\"}",
                 status, version);
    } else {
        snprintf(body, sizeof(body), "{\"ota_status\":\"%s\"}", status);
    }
    zc_err_t err = zc_report_send(body, strlen(body));
    if (err != ZC_OK) {
        LOGW("OTA status report to TB failed (err %d) - non-fatal", err);
    }
}

// Version comparison: "major.minor.patch" numeric ordering (MAJOR.MINOR.PATCH)
static bool is_newer_version(const char *server_ver, const char *running_ver)
{
    int s[3] = {0}, r[3] = {0};
    sscanf(server_ver,  "%d.%d.%d", &s[0], &s[1], &s[2]);
    sscanf(running_ver, "%d.%d.%d", &r[0], &r[1], &r[2]);
    for (int i = 0; i < 3; i++) {
        if (s[i] > r[i]) return true;
        if (s[i] < r[i]) return false;
    }
    return false;
}

static esp_err_t ota_http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
    case HTTP_EVENT_ON_HEADER:
        // LOGI("HTTP Header: %s: %s", evt->header_key, evt->header_value); // Uncomment to enable debug again
        break;
    default:
        break;
    }
    return ESP_OK;
}

// Component identifier sent in the OTA check POST body.
// Must match the component name registered in ota_manifest.
#define OTA_COMPONENT_NAME "zoefall_rx"

// Fetch version JSON from the OTA Edge Function and extract "version" + "url" fields.
// Returns heap-allocated JSON string on success, NULL on failure.
// Caller must free() the returned string.
static char *fetch_version_json(void)
{
    if (zc_zf_rx_cfg.ota_version_url[0] == '\0') {
        LOGW("OTA version URL not configured; skipping check");
        return NULL;
    }

    esp_http_client_config_t cfg = {
        .url = zc_zf_rx_cfg.ota_version_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = OTA_RECV_TIMEOUT_MS,
        .event_handler = ota_http_event_handler,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        LOGE("Failed to init HTTP client for version check");
        return NULL;
    }

    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    // Auth header, only if configured
    if (zc_zf_rx_cfg.ota_secret_key[0] != '\0') {
        esp_http_client_set_header(client, "x-sensor-api-key", zc_zf_rx_cfg.ota_secret_key);
    }

    // Ngrok bypass
    esp_http_client_set_header(client, "ngrok-skip-browser-warning", "1");

    // Build POST body at runtime to include sensor_id and cfg_ver for remote config
    cJSON *req_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(req_obj, "component", OTA_COMPONENT_NAME);
    cJSON_AddStringToObject(req_obj, "sensor_id", zc_zf_rx_cfg.sensor_id);
    cJSON_AddNumberToObject(req_obj, "cfg_ver", (double)zc_zf_rx_cfg.cfg_ver);
    char *post_body = cJSON_PrintUnformatted(req_obj);
    cJSON_Delete(req_obj);
    if (!post_body) {
        LOGE("Failed to build OTA request JSON");
        esp_http_client_cleanup(client);
        return NULL;
    }
    int post_len = (int)strlen(post_body);

    esp_err_t err = esp_http_client_open(client, post_len);
    if (err != ESP_OK) {
        LOGE("HTTP open failed: %s", esp_err_to_name(err));
        free(post_body);
        esp_http_client_cleanup(client);
        return NULL;
    }

    int written = esp_http_client_write(client, post_body, post_len);
    free(post_body);
    if (written < 0) {
        LOGE("HTTP write failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    int content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        LOGE("Failed to fetch headers");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        LOGW("Version endpoint returned HTTP %d", status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    // Cap read size to prevent unbounded allocation
    int buf_size = (content_length > 0 && content_length < 2048) ? content_length + 1 : 2048;
    char *buf = malloc(buf_size);
    if (!buf) {
        LOGE("malloc failed for version JSON");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    int read_len = esp_http_client_read(client, buf, buf_size - 1);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (read_len <= 0) {
        LOGE("Empty response from version endpoint");
        free(buf);
        return NULL;
    }
    buf[read_len] = '\0';
    return buf;
}

// OTA check task: periodically polls version.json and performs OTA if newer
static void ota_check_task(void *pvParameters)
{
    // Let the system stabilise: WiFi, CSI, first DAPD cycles
    vTaskDelay(pdMS_TO_TICKS(OTA_FIRST_CHECK_DELAY_MS));

    while (true) {
        const esp_app_desc_t *app_desc = esp_app_get_description();
        LOGI("Running firmware version: %s", app_desc->version);

        // 1. Fetch version metadata from server
        char *json_str = fetch_version_json();
        if (!json_str) {
            goto sleep;
        }

        // 2. Parse JSON
        cJSON *root = cJSON_Parse(json_str);
        if (!root) {
            LOGE("Failed to parse version JSON");
            free(json_str);
            goto sleep;
        }

        const char *new_ver = cJSON_GetStringValue(cJSON_GetObjectItem(root, "version"));
        const char *fw_url  = cJSON_GetStringValue(cJSON_GetObjectItem(root, "url"));

        // Copy new_ver to a stack buffer before cJSON_Delete() frees the tree.
        // Accessing new_ver after cJSON_Delete() is a use-after-free.
        char new_ver_buf[32] = {0};
        if (new_ver) strlcpy(new_ver_buf, new_ver, sizeof(new_ver_buf));

        // 3. Handle remote config update if present (before firmware version check).
        //    If a config update is applied we reboot immediately; firmware update
        //    will be picked up by the next poll after the reboot.
        cJSON *cfg_update = cJSON_GetObjectItem(root, "config_update");
        if (cJSON_IsObject(cfg_update)) {
            cJSON *cv   = cJSON_GetObjectItem(cfg_update, "config_version");
            cJSON *ckeys = cJSON_GetObjectItem(cfg_update, "keys");
            if (cJSON_IsNumber(cv) && cJSON_IsObject(ckeys)) {
                uint32_t new_cfg_ver = (uint32_t)cv->valuedouble;
                if (new_cfg_ver > zc_zf_rx_cfg.cfg_ver) {
                    LOGI("Remote config update available (cfg_ver %u -> %u)",
                         (unsigned)zc_zf_rx_cfg.cfg_ver, (unsigned)new_cfg_ver);
                    esp_err_t cfg_err = zc_zf_rx_cfg_apply_remote_update(ckeys, new_cfg_ver);
                    if (cfg_err == ESP_OK) {
                        char tb_body[64];
                        snprintf(tb_body, sizeof(tb_body),
                                 "{\"cfg_ver_applied\":%u}", (unsigned)new_cfg_ver);
                        zc_report_send(tb_body, strlen(tb_body));
                        vTaskDelay(pdMS_TO_TICKS(3000));  // let report queue flush
                        cJSON_Delete(root);
                        free(json_str);
                        LOGW("Config update applied, rebooting to reload NVS config");
                        esp_restart();
                    }
                }
            }
        }

        if (!new_ver || !fw_url) {
            LOGE("version.json missing 'version' or 'url' fields");
            cJSON_Delete(root);
            free(json_str);
            goto sleep;
        }

        // 4. Compare firmware versions
        if (!is_newer_version(new_ver, app_desc->version)) {
            LOGI("No update available (server=%s, running=%s)", new_ver, app_desc->version);
            cJSON_Delete(root);
            free(json_str);
            goto sleep;
        }

        LOGW("Update available: %s -> %s", app_desc->version, new_ver);
        LOGI("Firmware URL: %s", fw_url);

        // Report download start to ThingsBoard
        _report_ota_status("downloading", new_ver, NULL);

        // 4. Signal DAPD task to pause and notify backend
        ota_in_progress = true;
        vTaskDelay(pdMS_TO_TICKS(2000));  // Let current DAPD cycle finish
        zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                              zc_zf_rx_cfg.ota_secret_key, true);

        // 5. Perform HTTPS OTA
        esp_http_client_config_t http_cfg = {
            .url = fw_url,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .timeout_ms = OTA_RECV_TIMEOUT_MS,
            .buffer_size = OTA_HTTP_BUF_SIZE,
            .buffer_size_tx = 1024,
            .keep_alive_enable = true,
        };

        esp_https_ota_config_t ota_cfg = {
            .http_config = &http_cfg,
        };

        esp_https_ota_handle_t handle = NULL;
        esp_err_t err = esp_https_ota_begin(&ota_cfg, &handle);
        if (err != ESP_OK) {
            LOGE("OTA begin failed: %s", esp_err_to_name(err));
            ota_in_progress = false;
            zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                                  zc_zf_rx_cfg.ota_secret_key, false);
            cJSON_Delete(root);
            free(json_str);
            goto sleep;
        }

        // Validate image size before writing a single byte to flash.
        int image_size = esp_https_ota_get_image_size(handle);
        const esp_partition_t *target_partition = esp_ota_get_next_update_partition(NULL);
        if (image_size > 0 && target_partition != NULL &&
            (size_t)image_size > target_partition->size) {
            LOGE("OTA image (%d B) exceeds partition size (%lu B) - aborting",
                 image_size, (unsigned long)target_partition->size);
            esp_https_ota_abort(handle);
            _report_ota_status("size_exceeded", new_ver_buf, "image too large for partition");
            ota_in_progress = false;
            zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                                  zc_zf_rx_cfg.ota_secret_key, false);
            cJSON_Delete(root);
            free(json_str);
            goto sleep;
        }

        // Download loop with progress reporting
        int last_pct = -1;
        while (true) {
            err = esp_https_ota_perform(handle);
            if (err == ESP_ERR_HTTPS_OTA_IN_PROGRESS) {
                int total = esp_https_ota_get_image_size(handle);
                int read  = esp_https_ota_get_image_len_read(handle);
                int pct   = (total > 0) ? (read * 100 / total) : 0;
                if (pct / 10 > last_pct / 10) {
                    LOGI("Download: %d%% (%d / %d bytes)", pct, read, total);
                    last_pct = pct;
                }
                continue;
            }
            break;
        }

        if (err != ESP_OK) {
            LOGE("OTA download failed: %s", esp_err_to_name(err));
            esp_https_ota_abort(handle);
            ota_in_progress = false;
            zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                                  zc_zf_rx_cfg.ota_secret_key, false);
            cJSON_Delete(root);
            free(json_str);
            goto sleep;
        }

        if (!esp_https_ota_is_complete_data_received(handle)) {
            LOGE("Incomplete OTA data received");
            esp_https_ota_abort(handle);
            ota_in_progress = false;
            zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                                  zc_zf_rx_cfg.ota_secret_key, false);
            cJSON_Delete(root);
            free(json_str);
            goto sleep;
        }

        err = esp_https_ota_finish(handle);
        cJSON_Delete(root);
        free(json_str);

        if (err != ESP_OK) {
            LOGE("OTA finish failed: %s", esp_err_to_name(err));
            ota_in_progress = false;
            zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                                  zc_zf_rx_cfg.ota_secret_key, false);
            goto sleep;
        }

        LOGW("OTA successful! Rebooting in 3 seconds...");
        ota_in_progress = false;
        zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                              zc_zf_rx_cfg.ota_secret_key, false);
        _report_ota_status("success", new_ver_buf, NULL);
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();

sleep: ;
        vTaskDelay(pdMS_TO_TICKS(OTA_CHECK_INTERVAL_MS));
    }
}

// Safety watchdog: if the firmware is not validated within the timeout,
// force a rollback to the previous partition.
static void ota_safety_watchdog(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(OTA_VALIDATION_TIMEOUT_MS));

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            LOGE("Firmware not validated within %d ms - rolling back!",
                     OTA_VALIDATION_TIMEOUT_MS);
            _report_ota_status("rollback",
                               esp_app_get_description()->version,
                               "DAPD validation timeout");
            zc_maintenance_notify(zc_zf_rx_cfg.maintenance_url, zc_zf_rx_cfg.sensor_id,
                                  zc_zf_rx_cfg.ota_secret_key, false);
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
    }
    // Firmware was already validated or this is a fresh flash - just exit
    vTaskDelete(NULL);
}

// Public API
void ota_start_task(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    LOGI("Boot partition: %s at offset 0x%"PRIx32, running->label, running->address);

    xTaskCreate(ota_check_task,     "ota_check",  8192, NULL, 2, NULL);
    xTaskCreate(ota_safety_watchdog, "ota_wdog",  2048, NULL, 1, NULL);
}

void ota_mark_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            LOGI("Marking firmware as valid (cancelling rollback)");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

const char *ota_get_running_version(void)
{
    return esp_app_get_description()->version;
}

void ota_check_partition_layout(void)
{
    static const struct {
        const char    *label;
        uint32_t       expected_offset;
        uint32_t       expected_size;
    } expected[] = {
        { "nvs",        0x9000,  0x6000 },
        { "nvs_backup", 0xF000,  0x2000 },
        { "otadata",    0x11000, 0x2000 },
    };

    bool ok = true;
    for (int i = 0; i < (int)(sizeof(expected) / sizeof(expected[0])); i++) {
        const esp_partition_t *p = esp_partition_find_first(
            ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, expected[i].label);
        if (!p || p->address != expected[i].expected_offset
               || p->size    != expected[i].expected_size) {
            LOGE("Partition '%s' mismatch: found addr=0x%"PRIx32" size=0x%"PRIx32
                 ", expected addr=0x%"PRIx32" size=0x%"PRIx32,
                 expected[i].label,
                 p ? p->address : 0u, p ? p->size : 0u,
                 expected[i].expected_offset, expected[i].expected_size);
            ok = false;
        }
    }

    if (!ok) {
        const esp_partition_t *running = esp_ota_get_running_partition();
        esp_ota_img_states_t state;
        if (esp_ota_get_state_partition(running, &state) == ESP_OK &&
            state == ESP_OTA_IMG_PENDING_VERIFY) {
            LOGE("Partition mismatch on PENDING_VERIFY firmware - rolling back");
            esp_ota_mark_app_invalid_rollback_and_reboot();
        } else {
            LOGE("Partition mismatch on validated firmware - continuing with caution");
        }
    }
}
