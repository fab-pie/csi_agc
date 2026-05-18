#ifndef CONFIG_H
#define CONFIG_H

#include <esp_task_wdt.h>
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "esp_err.h"

#define LED_GPIO 21 // GPIO pin for the LED
#define WDT_TIMEOUT_MS 30 * 1000 // in ms // TODO: restore to (5 * 60 * 1000)

extern bool got_timestamp;

// Runtime config - loaded from NVS namespace "zoefall"
// NVS keys: tb_token, wifi_ssid, wifi_pass, up_ssid, up_pass, pair_id, report_s, csi_rate,
//           cfg_ver, up_ssid_new, up_pass_new, up_activation_ts
#include <stdint.h>
typedef struct {
    // --- loaded from NVS ---
    char     wifi_ap_ssid[32];      // AP SSID (e.g. "ZoeFall7DA4")
    char     wifi_ap_password[32];
    char     upstream_ssid[64];     // upstream WiFi SSID
    char     upstream_password[64];
    char     tb_access_token[64];
    char     supabase_url[128];     // Supabase base URL (e.g. "https://api.zoe.care")
    char     ota_secret_key[128];   // x-sensor-api-key for Edge Functions
    char     sensor_id[64];         // sensors.id UUID
    uint16_t report_status_delay;   // seconds between isAlive pings
    uint16_t csi_rate;              // Hz
    char     pair_id[8];            // last 4 hex digits of TX MAC, e.g. "7DA4"
    uint32_t cfg_ver;               // last applied remote config version (0 = none)
    // staged upstream credential switch (written by remote config, consumed by tx_cred_switch)
    char     upstream_ssid_new[64];
    char     upstream_password_new[64];
    uint32_t upstream_activation_ts; // 0 = apply immediately on boot, >0 = unix timestamp
    // --- computed after load, not stored ---
    char     tb_url[128];           // "https://iot.zoe.care/api/v1/<token>/telemetry"
    char     ota_version_url[192];  // "<supabase_url>/functions/v1/check-ota"
    char     maintenance_url[192];  // "<supabase_url>/functions/v1/mark-maintenance"
} zc_zf_tx_cfg_t;

extern zc_zf_tx_cfg_t zc_zf_tx_cfg;
void zc_zf_tx_cfg_load(void);
void zc_zf_tx_cfg_print(void);

// Apply a remote config update received from check-ota.
// keys: cJSON object whose children are the NVS key/value pairs to write.
// new_cfg_ver: the config_version from the server response.
// Writes allowlisted keys to main NVS and mirrors to nvs_backup.
// Writes cfg_ver last so a mid-update crash retries on the next poll.
// Returns ESP_OK if at least one key was applied, ESP_ERR_NOT_FOUND otherwise.
esp_err_t zc_zf_tx_cfg_apply_remote_update(const cJSON *keys, uint32_t new_cfg_ver);

#endif /* CONFIG_H */