/*
 * zoefall_tx/src/config.c
 *
 * Runtime config loaded from NVS namespace "zoefall".
 * NVS keys (all ≤15 chars):
 *   tb_token   string  ThingsBoard access token
 *   wifi_ssid  string  AP SSID  (e.g. "ZoeFall7DA4")
 *   wifi_pass  string  AP password
 *   up_ssid    string  Upstream WiFi SSID
 *   up_pass    string  Upstream WiFi password
 *   pair_id    string  Last 4 hex digits of TX MAC
 *   report_s   u16     isAlive interval in seconds
 *   csi_rate   u16     CSI rate in Hz
 */

// printf is used here (not ESP_LOG) so output is visible before log subsystem init.

// -------------------------------------------------------------------
#pragma region INCLUDES
// -------------------------------------------------------------------

#include <stdio.h>
#include <stddef.h>
#include <string.h>

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"

#include "zoecare/tx/config.h"
#include "esp_log.h"
#include "cJSON.h"
#include "zc_debug.h"

// -------------------------------------------------------------------
#pragma region GLOBALS
// -------------------------------------------------------------------

zc_zf_tx_cfg_t zc_zf_tx_cfg = {0};

// -------------------------------------------------------------------
#pragma region HELPERS
// -------------------------------------------------------------------

#define NVS_NAMESPACE "zoefall"

static esp_err_t _nvs_get_str_safe(nvs_handle_t h, const char *key,
                                    char *dest, size_t dest_size)
{
    size_t required = 0;
    esp_err_t err = nvs_get_str(h, key, NULL, &required);
    if (err != ESP_OK) return err;
    if (required > dest_size) {
        printf("config: NVS key '%s' value too long (%u > %u), truncating\n",
               key, (unsigned)required, (unsigned)dest_size);
        required = dest_size;
    }
    return nvs_get_str(h, key, dest, &required);
}

/*
 * Mirror a single key/value into the nvs_backup partition.
 * Called after any runtime write to main NVS so backup stays in sync.
 * Supported types: NVS_TYPE_STR, NVS_TYPE_U16, NVS_TYPE_U32.
 */
static esp_err_t _sync_to_backup(const char *ns, const char *key,
                                  const void *value, nvs_type_t type)
{
    const esp_partition_t *bkp =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x41, "nvs_backup");
    if (!bkp) return ESP_ERR_NOT_FOUND;

    esp_err_t err = nvs_flash_init_partition_ptr(bkp);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
        return err;
    if (err != ESP_OK) return err;

    nvs_handle_t h;
    err = nvs_open_from_partition(bkp->label, ns, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        nvs_flash_deinit_partition(bkp->label);
        return err;
    }

    switch (type) {
        case NVS_TYPE_STR: err = nvs_set_str(h, key, (const char *)value); break;
        case NVS_TYPE_U16: err = nvs_set_u16(h, key, *(const uint16_t *)value); break;
        case NVS_TYPE_U32: err = nvs_set_u32(h, key, *(const uint32_t *)value); break;
        default:           err = ESP_ERR_INVALID_ARG; break;
    }
    if (err == ESP_OK) nvs_commit(h);
    nvs_close(h);
    nvs_flash_deinit_partition(bkp->label);
    return err;
}

static void _compute_derived_urls(zc_zf_tx_cfg_t *cfg)
{
    snprintf(cfg->tb_url, sizeof(cfg->tb_url),
             "https://iot.zoe.care/api/v1/%s/telemetry",
             cfg->tb_access_token);

    snprintf(cfg->ota_version_url, sizeof(cfg->ota_version_url),
             "%s/functions/v1/check-ota", cfg->supabase_url);

    snprintf(cfg->maintenance_url, sizeof(cfg->maintenance_url),
             "%s/functions/v1/mark-maintenance", cfg->supabase_url);
}

static void _safe_mode_loop(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    printf("\n");
    printf("=============================================================\n");
    printf("  NVS CONFIG MISSING - SAFE MODE (TX)\n");
    printf("  Device MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("  Run the provisioner to re-flash this device:\n");
    printf("    python scripts/main.py -> provisioner -> provision\n");
    printf("  Or use reprovision --mac %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("=============================================================\n");

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    while (true) {
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// -------------------------------------------------------------------
#pragma region NVS LOAD
// -------------------------------------------------------------------

static esp_err_t _load_from_handle(nvs_handle_t h, zc_zf_tx_cfg_t *cfg)
{
    esp_err_t err;

#define NVS_STR(key, field)                                              \
    err = _nvs_get_str_safe(h, key, cfg->field, sizeof(cfg->field));    \
    if (err != ESP_OK) {                                                 \
        printf("config: NVS key '%s' missing or error: %s\n",           \
               key, esp_err_to_name(err));                              \
        return err;                                                      \
    }

    NVS_STR("tb_token",  tb_access_token)
    NVS_STR("wifi_ssid", wifi_ap_ssid)
    NVS_STR("wifi_pass", wifi_ap_password)
    NVS_STR("up_ssid",   upstream_ssid)
    NVS_STR("up_pass",   upstream_password)
    NVS_STR("supa_url",  supabase_url)
    NVS_STR("ota_key",   ota_secret_key)
    NVS_STR("sensor_id", sensor_id)

#undef NVS_STR

    /* Optional: report_s (default 60 s) */
    uint16_t report_s = 60;
    err = nvs_get_u16(h, "report_s", &report_s);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("config: NVS key 'report_s' error: %s\n", esp_err_to_name(err));
    }
    cfg->report_status_delay = report_s;

    /* Optional: csi_rate (default 100 Hz) */
    uint16_t csi_rate = 100;
    err = nvs_get_u16(h, "csi_rate", &csi_rate);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("config: NVS key 'csi_rate' error: %s\n", esp_err_to_name(err));
    }
    cfg->csi_rate = csi_rate;

    /* Optional: pair_id */
    _nvs_get_str_safe(h, "pair_id", cfg->pair_id, sizeof(cfg->pair_id));

    /* cfg_ver: optional, default 0 */
    uint32_t cfg_ver = 0;
    err = nvs_get_u32(h, "cfg_ver", &cfg_ver);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        printf("config: NVS key 'cfg_ver' error: %s\n", esp_err_to_name(err));
    }
    cfg->cfg_ver = cfg_ver;

    /* Staged upstream switch keys: optional, default empty */
    _nvs_get_str_safe(h, "up_ssid_new",  cfg->upstream_ssid_new,      sizeof(cfg->upstream_ssid_new));
    _nvs_get_str_safe(h, "up_pass_new",  cfg->upstream_password_new,   sizeof(cfg->upstream_password_new));
    uint32_t up_activation_ts = 0;
    nvs_get_u32(h, "up_activation_ts", &up_activation_ts);
    cfg->upstream_activation_ts = up_activation_ts;

    return ESP_OK;
}

static esp_err_t _load_from_main_nvs(zc_zf_tx_cfg_t *cfg)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        printf("config: nvs_open('%s') failed: %s\n",
               NVS_NAMESPACE, esp_err_to_name(err));
        return err;
    }
    err = _load_from_handle(h, cfg);
    nvs_close(h);
    return err;
}

// -------------------------------------------------------------------
#pragma region NVS BACKUP RESTORE
// -------------------------------------------------------------------

static esp_err_t _restore_from_backup(void)
{
    const esp_partition_t *bkp =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x41, "nvs_backup");
    if (!bkp) {
        printf("config: nvs_backup partition not found - cannot restore\n");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = nvs_flash_init_partition_ptr(bkp);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("config: nvs_backup partition is corrupt (%s)\n",
               esp_err_to_name(err));
        return err;
    }
    if (err != ESP_OK) {
        printf("config: nvs_flash_init_partition_ptr(nvs_backup) failed: %s\n",
               esp_err_to_name(err));
        return err;
    }

    nvs_handle_t src = 0, dst = 0;
    bool all_ok = false;

    err = nvs_open_from_partition(bkp->label, NVS_NAMESPACE, NVS_READONLY, &src);
    if (err != ESP_OK) {
        printf("config: cannot open backup namespace: %s\n", esp_err_to_name(err));
        goto cleanup_partition;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &dst);
    if (err != ESP_OK) {
        printf("config: cannot open main NVS for writing: %s\n", esp_err_to_name(err));
        nvs_close(src);
        goto cleanup_partition;
    }

    static const struct { const char *key; bool required; } _keys[] = {
        { "tb_token",  true  },
        { "wifi_ssid", true  },
        { "wifi_pass", true  },
        { "up_ssid",   true  },
        { "up_pass",   true  },
        { "supa_url",  true  },
        { "ota_key",   true  },
        { "sensor_id", true  },
        { "pair_id",   false },
        { "report_s",  false },
        { "csi_rate",  false },
        { "cfg_ver",          false },
        { "up_ssid_new",      false },
        { "up_pass_new",      false },
        { "up_activation_ts", false },
    };

    all_ok = true;
    char buf[256];

    for (size_t i = 0; i < sizeof(_keys) / sizeof(_keys[0]); i++) {
        const char *key = _keys[i].key;

        size_t len = sizeof(buf);
        esp_err_t e = nvs_get_str(src, key, buf, &len);
        if (e == ESP_OK) {
            e = nvs_set_str(dst, key, buf);
            if (e != ESP_OK) {
                printf("config: restore: nvs_set_str('%s') failed: %s\n",
                       key, esp_err_to_name(e));
                all_ok = false;
            }
            continue;
        }

        /* u16 keys */
        if (strcmp(key, "report_s") == 0 || strcmp(key, "csi_rate") == 0) {
            uint16_t v = 0;
            e = nvs_get_u16(src, key, &v);
            if (e == ESP_OK) nvs_set_u16(dst, key, v);
            continue;
        }

        /* u32 keys */
        if (strcmp(key, "cfg_ver") == 0 || strcmp(key, "up_activation_ts") == 0) {
            uint32_t v = 0;
            e = nvs_get_u32(src, key, &v);
            if (e == ESP_OK) nvs_set_u32(dst, key, v);
            continue;
        }

        if (_keys[i].required) {
            printf("config: restore: required key '%s' missing in backup: %s\n",
                   key, esp_err_to_name(e));
            all_ok = false;
        }
    }

    if (all_ok) {
        err = nvs_commit(dst);
        if (err == ESP_OK)
            printf("config: NVS restored from backup partition successfully\n");
        else
            printf("config: nvs_commit after restore failed: %s\n", esp_err_to_name(err));
    }

    nvs_close(src);
    nvs_close(dst);

cleanup_partition:
    nvs_flash_deinit_partition(bkp->label);
    return all_ok ? ESP_OK : ESP_FAIL;
}

// -------------------------------------------------------------------
#pragma region FACTORY RESTORE
// -------------------------------------------------------------------

static esp_err_t _restore_from_factory(void)
{
    const esp_partition_t *fct =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                 ESP_PARTITION_SUBTYPE_DATA_NVS, "fctry");
    if (!fct) {
        printf("config: fctry partition not found - skipping factory restore\n");
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = nvs_flash_init_partition_ptr(fct);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        printf("config: fctry partition is corrupt (%s)\n", esp_err_to_name(err));
        return err;
    }
    if (err != ESP_OK) {
        printf("config: nvs_flash_init_partition_ptr(fctry) failed: %s\n",
               esp_err_to_name(err));
        return err;
    }

    nvs_handle_t src = 0, dst = 0;
    bool all_ok = false;

    err = nvs_open_from_partition(fct->label, NVS_NAMESPACE, NVS_READONLY, &src);
    if (err != ESP_OK) {
        printf("config: cannot open fctry namespace: %s\n", esp_err_to_name(err));
        goto cleanup;
    }

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &dst);
    if (err != ESP_OK) {
        printf("config: cannot open main NVS for writing: %s\n", esp_err_to_name(err));
        nvs_close(src);
        goto cleanup;
    }

    /* Factory only contains provisioned keys - no cfg_ver or staged switch keys */
    static const struct { const char *key; bool required; } _keys[] = {
        { "tb_token",  true  },
        { "wifi_ssid", true  },
        { "wifi_pass", true  },
        { "up_ssid",   true  },
        { "up_pass",   true  },
        { "supa_url",  true  },
        { "ota_key",   true  },
        { "sensor_id", true  },
        { "pair_id",   false },
        { "report_s",  false },
        { "csi_rate",  false },
    };

    all_ok = true;
    char buf[256];

    for (size_t i = 0; i < sizeof(_keys) / sizeof(_keys[0]); i++) {
        const char *key = _keys[i].key;
        size_t len = sizeof(buf);
        esp_err_t e = nvs_get_str(src, key, buf, &len);
        if (e == ESP_OK) {
            e = nvs_set_str(dst, key, buf);
            if (e != ESP_OK) {
                printf("config: factory restore: nvs_set_str('%s') failed: %s\n",
                       key, esp_err_to_name(e));
                all_ok = false;
            }
            continue;
        }
        if (strcmp(key, "report_s") == 0 || strcmp(key, "csi_rate") == 0) {
            uint16_t v = 0;
            e = nvs_get_u16(src, key, &v);
            if (e == ESP_OK) nvs_set_u16(dst, key, v);
            continue;
        }
        if (_keys[i].required) {
            printf("config: factory restore: required key '%s' missing: %s\n",
                   key, esp_err_to_name(e));
            all_ok = false;
        }
    }

    if (all_ok) {
        err = nvs_commit(dst);
        if (err == ESP_OK)
            printf("config: NVS restored from factory partition successfully\n");
        else
            printf("config: nvs_commit after factory restore failed: %s\n",
                   esp_err_to_name(err));
    }

    nvs_close(src);
    nvs_close(dst);

cleanup:
    nvs_flash_deinit_partition(fct->label);
    return all_ok ? ESP_OK : ESP_FAIL;
}

// -------------------------------------------------------------------
#pragma region PUBLIC API
// -------------------------------------------------------------------

void zc_zf_tx_cfg_load(void)
{
    memset(&zc_zf_tx_cfg, 0, sizeof(zc_zf_tx_cfg));

    /* 1. Try main NVS */
    esp_err_t err = _load_from_main_nvs(&zc_zf_tx_cfg);
    if (err == ESP_OK) {
        printf("config: loaded from NVS (namespace '%s')\n", NVS_NAMESPACE);
        _compute_derived_urls(&zc_zf_tx_cfg);
        return;
    }

    /* 2. Try backup restore */
    printf("config: main NVS load failed (%s) - attempting backup restore...\n",
           esp_err_to_name(err));

    err = _restore_from_backup();
    if (err == ESP_OK) {
        err = _load_from_main_nvs(&zc_zf_tx_cfg);
        if (err == ESP_OK) {
            printf("config: loaded from NVS after backup restore\n");
            _compute_derived_urls(&zc_zf_tx_cfg);
            return;
        }
    }

    /* 3. Try factory restore */
    printf("config: backup restore failed - attempting factory restore...\n");

    err = _restore_from_factory();
    if (err == ESP_OK) {
        err = _load_from_main_nvs(&zc_zf_tx_cfg);
        if (err == ESP_OK) {
            printf("config: loaded from NVS after factory restore\n");
            _compute_derived_urls(&zc_zf_tx_cfg);
            return;
        }
    }

    /* 4. All paths failed - safe mode */
    printf("config: FATAL - NVS, backup and factory all unusable. Entering safe mode.\n");
    _safe_mode_loop();
}

void zc_zf_tx_cfg_print(void)
{
    ZC_IF_DEBUG_MON {
        printf("zc_zf_tx_cfg:\n");
        printf("\twifi_ap_ssid:         %s\n", zc_zf_tx_cfg.wifi_ap_ssid);
        printf("\twifi_ap_password:     %s\n", zc_zf_tx_cfg.wifi_ap_password);
        printf("\tupstream_ssid:        %s\n", zc_zf_tx_cfg.upstream_ssid);
        printf("\tupstream_password:    %s\n", zc_zf_tx_cfg.upstream_password);
        printf("\ttb_access_token:      %s\n", zc_zf_tx_cfg.tb_access_token);
        printf("\tsupabase_url:         %s\n", zc_zf_tx_cfg.supabase_url);
        printf("\tota_secret_key:       %.20s...\n", zc_zf_tx_cfg.ota_secret_key);
        printf("\tsensor_id:            %s\n", zc_zf_tx_cfg.sensor_id);
        printf("\treport_status_delay:  %u s\n", (unsigned)zc_zf_tx_cfg.report_status_delay);
        printf("\tcsi_rate:             %u Hz\n", (unsigned)zc_zf_tx_cfg.csi_rate);
        printf("\tpair_id:              %s\n", zc_zf_tx_cfg.pair_id);
        printf("\tcfg_ver:              %u\n", (unsigned)zc_zf_tx_cfg.cfg_ver);
        if (zc_zf_tx_cfg.upstream_ssid_new[0])
            printf("\tupstream_ssid_new:    %s (activation_ts=%u)\n",
                   zc_zf_tx_cfg.upstream_ssid_new,
                   (unsigned)zc_zf_tx_cfg.upstream_activation_ts);
        printf("\t[computed] tb_url:         %s\n", zc_zf_tx_cfg.tb_url);
        printf("\t[computed] ota_version_url: %s\n", zc_zf_tx_cfg.ota_version_url);
    }
}

// -------------------------------------------------------------------
#pragma region REMOTE CONFIG
// -------------------------------------------------------------------

static const struct { const char *key; nvs_type_t type; } _tx_allowlist[] = {
    { "tb_token",         NVS_TYPE_STR },
    { "sensor_id",        NVS_TYPE_STR },
    { "wifi_ssid",        NVS_TYPE_STR },
    { "wifi_pass",        NVS_TYPE_STR },
    { "up_ssid",          NVS_TYPE_STR },
    { "up_pass",          NVS_TYPE_STR },
    { "supa_url",         NVS_TYPE_STR },
    { "ota_key",          NVS_TYPE_STR },
    { "pair_id",          NVS_TYPE_STR },
    { "report_s",         NVS_TYPE_U16 },
    { "csi_rate",         NVS_TYPE_U16 },
    { "up_ssid_new",      NVS_TYPE_STR },
    { "up_pass_new",      NVS_TYPE_STR },
    { "up_activation_ts", NVS_TYPE_U32 },
};

esp_err_t zc_zf_tx_cfg_apply_remote_update(const cJSON *keys, uint32_t new_cfg_ver)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE("config", "TX apply_remote_update: nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    int applied = 0;
    int rejected = 0;

    cJSON *item = keys->child;
    while (item) {
        const char *key = item->string;
        nvs_type_t expected_type = NVS_TYPE_ANY;

        for (size_t i = 0; i < sizeof(_tx_allowlist) / sizeof(_tx_allowlist[0]); i++) {
            if (strcmp(_tx_allowlist[i].key, key) == 0) {
                expected_type = _tx_allowlist[i].type;
                break;
            }
        }

        if (expected_type == NVS_TYPE_ANY) {
            ESP_LOGW("config", "TX remote_update: key '%s' not in TX allowlist, skipping", key);
            rejected++;
            item = item->next;
            continue;
        }

        esp_err_t e = ESP_OK;
        if (expected_type == NVS_TYPE_STR) {
            if (!cJSON_IsString(item)) {
                ESP_LOGW("config", "TX remote_update: key '%s' expected string", key);
                rejected++;
                item = item->next;
                continue;
            }
            e = nvs_set_str(h, key, item->valuestring);
            if (e == ESP_OK) _sync_to_backup(NVS_NAMESPACE, key, item->valuestring, NVS_TYPE_STR);
        } else if (expected_type == NVS_TYPE_U16) {
            if (!cJSON_IsNumber(item)) {
                ESP_LOGW("config", "TX remote_update: key '%s' expected number", key);
                rejected++;
                item = item->next;
                continue;
            }
            uint16_t v = (uint16_t)item->valuedouble;
            e = nvs_set_u16(h, key, v);
            if (e == ESP_OK) _sync_to_backup(NVS_NAMESPACE, key, &v, NVS_TYPE_U16);
        } else if (expected_type == NVS_TYPE_U32) {
            if (!cJSON_IsNumber(item)) {
                ESP_LOGW("config", "TX remote_update: key '%s' expected number", key);
                rejected++;
                item = item->next;
                continue;
            }
            uint32_t v = (uint32_t)item->valuedouble;
            e = nvs_set_u32(h, key, v);
            if (e == ESP_OK) _sync_to_backup(NVS_NAMESPACE, key, &v, NVS_TYPE_U32);
        }

        if (e != ESP_OK) {
            ESP_LOGW("config", "TX remote_update: nvs_set('%s') failed: %s", key, esp_err_to_name(e));
            rejected++;
        } else {
            ESP_LOGI("config", "TX remote_update: applied key '%s'", key);
            applied++;
        }
        item = item->next;
    }

    if (applied == 0) {
        ESP_LOGW("config", "TX remote_update: no keys applied, not bumping cfg_ver");
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }

    err = nvs_set_u32(h, "cfg_ver", new_cfg_ver);
    if (err == ESP_OK) {
        nvs_commit(h);
        _sync_to_backup(NVS_NAMESPACE, "cfg_ver", &new_cfg_ver, NVS_TYPE_U32);
        zc_zf_tx_cfg.cfg_ver = new_cfg_ver;
    } else {
        ESP_LOGW("config", "TX remote_update: failed to write cfg_ver: %s", esp_err_to_name(err));
    }

    nvs_close(h);
    ESP_LOGI("config", "TX remote_update: applied %d keys, rejected %d, cfg_ver now %u",
             applied, rejected, (unsigned)new_cfg_ver);
    return ESP_OK;
}
