#ifndef CONFIG_H
#define CONFIG_H

#include "esp_http_client.h"
#include "esp_log.h"
#include <math.h>
#include "zoecare/common/config.h"

/*  FIRMWARE CONFIGURATION
 *
 *  Compile-time toggles (DAPD_DIAG_METRICS, CSI_RAW_STREAM, etc.)
 *  are in main/CMakeLists.txt
 */

//  1. WiFi & Network
#define WIFI_STATIC_IP_LAST_OCTET  2        // Static IP: 192.168.4.<this> (used when WIFI_USE_DHCP=0)
#define WIFI_USE_DHCP              1        // 1 = DHCP (mobile hotspot / double-NAT), 0 = static IP (direct router)

//  2. CSI Acquisition
#define CSI_COLS_SIZE      64               // Subcarriers per packet (802.11n, fixed)

// DAPD_EXTENDED_WINDOW:
//   0 = T=400, K=13, 1 inference. Script: visualize_svm_realtime.py, compare_realtime.py
//   1 = T=800, K=29, 17 sliding inferences. Script: multi_score_realtime.py, compare_realtime.py --extended
//
// CSI_RAW_STREAM (in CMakeLists.txt):
//   0 = default, no AMP: stream
//   1 = enables AMP: ASCII stream at 100 Hz (required for compare_realtime.py and dapd_realtime.py)
//
// Configuration matrix:
//   Demo wireless:         EXTENDED=1, OUTPUT_TB,   CSI_RAW_STREAM=0 → Dashboard TB
//   Debug multi-score:     EXTENDED=1, OUTPUT_UART, CSI_RAW_STREAM=0 → multi_score_realtime.py
//   Validation extended:   EXTENDED=1, OUTPUT_UART, CSI_RAW_STREAM=1 → compare_realtime.py --extended
//   Validation standard:   EXTENDED=0, OUTPUT_UART, CSI_RAW_STREAM=1 → compare_realtime.py
//   Single score display:  EXTENDED=0, OUTPUT_UART, CSI_RAW_STREAM=0 → visualize_svm_realtime.py
//   DAPD heatmap:          EXTENDED=0, OUTPUT_UART, CSI_RAW_STREAM=1 → dapd_realtime.py
#define DAPD_EXTENDED_WINDOW  1

#if DAPD_EXTENDED_WINDOW
  #define CSI_WINDOW_SIZE      800          // 8 s at 100 Hz
  #define SVM_SLIDING_K_TOTAL  29           // Td/delta = 735/25
#else
  #define CSI_WINDOW_SIZE      400          // 4 s at 100 Hz (original)
  #define SVM_SLIDING_K_TOTAL  13           // = SVM_DAPD_K (from svm_params.h), single inference
#endif

#define CSI_CACHE_SIZE         1200         // Circular buffer rows (>= CSI_WINDOW_SIZE)
#define SLIDING_STEP           100          // SVM inference every N packets (100 = ~1 Hz)
#define SVM_SLIDING_N_INFER    (SVM_SLIDING_K_TOTAL - 13 + 1)  // 17 or 1 (13 = SVM_DAPD_K)

//  3. SVM Inference

//  Model parameters (gamma, intercept, n_features, n_sv) are auto-generated in svm_params.h, do not set them here.
#define SVM_THRESHOLD      0.0f             // Decision threshold.
                                            //   0.0 = raw decision boundary
                                            //   Model trained with 0.4
                                            //   Python tools default to 0.2

//  Output mode: where to send SVM results
#define OUTPUT_UART   0                     // UART only (for Python tools via serial)
#define OUTPUT_TB     1                     // ThingsBoard only (wireless demo, no PC)
#define OUTPUT_BOTH   2                     // Both UART and ThingsBoard

#define OUTPUT_MODE   OUTPUT_TB

//  4. Hardware
#define LED_STATUS_PIN     GPIO_NUM_2       // GPIO for status LED

//  5. Telemetry
#define TELEMETRY_MAX_ERRORS        10      // Consecutive errors before restart
#define TELEMETRY_ERROR_QUEUE_SIZE  10      // FreeRTOS queue depth

// Runtime config - loaded from NVS namespace "zoefall"
// NVS keys: tb_token, sensor_id, supa_url, ota_key, wifi_ssid, wifi_pass, pair_id, report_s, cfg_ver
#include <stdint.h>
#include "cJSON.h"
typedef struct {
    // --- loaded from NVS ---
    char     wifi_ap_ssid[32];
    char     wifi_ap_password[32];
    uint16_t report_status_delay;   // seconds between isAlive pings
    char     tb_access_token[64];
    char     sensor_id[64];         // Supabase sensors.id UUID
    char     supabase_url[128];     // e.g. "https://api.zoe.care"
    char     ota_secret_key[128];   // x-sensor-api-key for Edge Functions
    char     pair_id[8];            // last 4 hex digits of TX MAC, e.g. "7DA4"
    uint32_t cfg_ver;               // last applied remote config version (0 = none)
    // --- computed after load, not stored ---
    char     tb_url[128];           // "https://iot.zoe.care/api/v1/<token>/telemetry"
    char     supabase_alert_url[192]; // "<supabase_url>/functions/v1/on-new-alert"
    char     ota_version_url[192];    // "<supabase_url>/functions/v1/check-ota"
    char     maintenance_url[192];    // "<supabase_url>/functions/v1/mark-maintenance"
} zc_zf_rx_cfg_t;

extern zc_zf_rx_cfg_t zc_zf_rx_cfg;
void zc_zf_rx_cfg_load(void);
void zc_zf_rx_cfg_print(void);

// Apply a remote config update received from check-ota.
// keys: cJSON object whose children are the NVS key/value pairs to write.
// new_cfg_ver: the config_version from the server response.
// Writes allowlisted keys to main NVS and mirrors to nvs_backup.
// Writes cfg_ver last so a mid-update crash retries on the next poll.
// Returns ESP_OK if at least one key was applied, ESP_ERR_NOT_FOUND otherwise.
esp_err_t zc_zf_rx_cfg_apply_remote_update(const cJSON *keys, uint32_t new_cfg_ver);

//  6. Fall Validation (post-fall motion-absence confirmation)
//     Defaults are in fall_validation.h; uncomment here to override.
#define FV_ENABLED            1     // 0 = disabled, 1 = enabled
// #define FV_T_WAIT          2.0f  // Seconds before observation window
// #define FV_T_OBS           45.0f // Observation window duration (seconds)
// #define FV_F_STILL         0.60f // Required fraction of "still" observations
// #define FV_PERCENTILE      25    // Percentile for motion-absence threshold

//  7. OTA Update
//  SINGLE KNOB: change OTA_CHECK_INTERVAL_MS only when switching dev <-> production.
//  All other timings are derived as multiples. Production value: (15 * 60 * 1000).
#define OTA_CHECK_INTERVAL_MS     (30 * 1000)          // TODO: (15 * 60 * 1000) for production
#define OTA_RECV_TIMEOUT_MS       30000                // fixed: HTTP receive timeout per chunk
#define OTA_HTTP_BUF_SIZE         4096                 // fixed: download buffer size
#define OTA_FIRST_CHECK_DELAY_MS  (OTA_CHECK_INTERVAL_MS * 2)   // first check after boot
#define OTA_VALIDATION_TIMEOUT_MS (OTA_CHECK_INTERVAL_MS * 10)  // safety watchdog timeout

#endif /* CONFIG_H */
