// TODO optimisation : remplacer les divisions à répétition par des multiplicaitons de l'inverse.

/*
    logs
*/
#define TAG "ZF_RX"
#define FILE_ID 0x1C
#include "zoecare/logs/logs.h"

#include <stdlib.h>
#include <string.h>
#include <esp_task_wdt.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/ip4_addr.h"
#include <esp_err.h>
#include "driver/gpio.h"
#include "zoecare/rx/config.h"
#include "zoecare/rx/err.h"
#include "zoecare/err/err.h"
#include "zoecare/rx/svm_rbf.h"
#include "zoecare/rx/utils.h"
#include "zoecare/rx/version_info.h"
#include <math.h>
#include "assert.h"
#include "esp_csi_gain_ctrl.h"
#include "zoecare/rx/dapd.h"
#include "zoecare/rx/fall_validation.h"
#include "zoecare/rx/ota_update.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include <unistd.h>
#include "zoecare/wifi/wifi.h"
#include "zoecare/report/report.h"

// Chunk size for WDT-safe long sleeps. WDT timeout is 15s; 5s = 1/3 margin.
#define RX_WDT_CHUNK_MS  5000

/* ── Global Stdout Mutex to prevent binary and ESP_LOG interleaving ──────── */
SemaphoreHandle_t stdout_mutex = NULL;

int locked_vprintf(const char *fmt, va_list args) {
    int res = 0;
    if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);
    res = vprintf(fmt, args);
    fflush(stdout);
    if (stdout_mutex) xSemaphoreGive(stdout_mutex);
    return res;
}

void safe_binary_write(const uint8_t* data, size_t len) {
    if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);
    fwrite(data, 1, len, stdout);
    fflush(stdout);
    if (stdout_mutex) xSemaphoreGive(stdout_mutex);
}
/* ────────────────────────────────────────────────────────────────────────── */

// Includes for VFS to disable CRLF conversion on ALL console types
#include "esp_vfs_dev.h"
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
#include "esp_vfs_usb_serial_jtag.h"
#endif
#if CONFIG_ESP_CONSOLE_USB_CDC
#include "esp_vfs_cdcacm.h"
#endif



size_t csi_count;
signed short int *csi_cache = NULL;
int row_now = 0; // Points to the oldest row in the cache
SemaphoreHandle_t csi_mutex = NULL; // Protects csi_cache, row_now, packet_counter

int packet_counter = 1;
int rssi = 1;
bool led_flag = false;
unsigned int last_ts = 0;

/* ── CSI UART streaming ──────────────────────────────────────────────────── */
typedef struct {
    uint16_t seq;
    uint32_t ts_us;
    int16_t  amp[CSI_COLS_SIZE];
} csi_uart_pkt_t;

static QueueHandle_t s_csi_uart_q  = NULL;
static uint16_t      s_csi_seq     = 0;

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

#include "esp_sntp.h"

esp_http_client_config_t config; // HTTP client configuration (passed to zc_report_send_task)
esp_http_client_handle_t client_supabase;
esp_http_client_config_t config_supabase;

int pred_cache[10] = {0,0,0,0,0,0,0,0,0,0};

int low_csi = 0;                      // Counter for low CSI values
unsigned int is_alive_counter = 0;      // Number of isAlive messages sent
size_t csi_last_isAlive = 0;          // Last CSI count for isAlive
bool got_timestamp = false; // Indicate if the timestamp has been obtained

void send_supabase(const char *payload, const char *tag) {
    LOGI("Supabase payload: %s", payload);

    // Set headers and POST data
    esp_http_client_set_header(client_supabase, "Content-Type", "application/json");
    esp_http_client_set_header(client_supabase, "x-sensor-api-key", zc_zf_rx_cfg.ota_secret_key);
    esp_http_client_set_post_field(client_supabase, payload, strlen(payload));

    bool send = true;
    esp_err_t err =
        esp_http_client_perform(client_supabase); // Execute the HTTP POST request
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client_supabase);
        if (status_code == 200) {
            LOGI(
                        "supabase request successful, status code: %d",
                        status_code);
        } else if (status_code == 400) {
            char buffer[128];
            int read_len = esp_http_client_read_response(client_supabase, buffer, 128);
            if (read_len > 0) {
                buffer[read_len] = '\0';
                LOGE("Error Detail: %s", buffer);
            }
        } else {
            send = false;
            LOGE("Failed to send supabase message, error code : %d", status_code);
        }
    } else {
        send = false;
        LOGE("HTTP request failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client_supabase);
        client_supabase = esp_http_client_init(&config_supabase);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!send) {
        LOGE("Failed to send to supabase, message: %s", payload);
    }

}


void send_svm_scores(const float *svm_scores, int n_scores, int64_t dapd_time, int64_t svm_time)
{
    int size_message = 384;
    char *message = (char *)checked_malloc(size_message);
    if (!message) return;
    int ptr = snprintf(message, size_message, "{\"scores\":[");

    for (int i = 0; i < n_scores && ptr < size_message - 60; i++)
        ptr += snprintf(message + ptr, size_message - ptr,
                        "%.4f%s", svm_scores[i], (i < n_scores - 1) ? "," : "");

    snprintf(message + ptr, size_message - ptr, "], \"dapd_time_us\": %lld, \"svm_time_us\": %lld}", dapd_time, svm_time);
    LOGI("SVM payload: %s", message);
    ZC_TRY(zc_report_send, message, strlen(message));
    checked_free(message);
}


#if FV_ENABLED
/* Detailed telemetry for both TB dashboards (replaces send_svm_scores when FV active).
 * Single JSON payload per cycle containing: 17 scores, motion indicator,
 * compound rule result, decision metrics, validation state, timing. */
void send_validation_telemetry(const float *scores, int n_scores,
                               float motion_std, const fv_engine_t *eng,
                               const fv_compound_result_t *cr,
                               int64_t dapd_us, int64_t svm_us)
{
    int size_msg = 800;
    char *msg = (char *)checked_malloc(size_msg);
    if (!msg) return;

    int ptr = snprintf(msg, size_msg, "{\"scores\":[");
    for (int i = 0; i < n_scores && ptr < size_msg - 300; i++)
        ptr += snprintf(msg + ptr, size_msg - ptr,
                        "%.4f%s", scores[i], (i < n_scores - 1) ? "," : "");

    // Live baseline P25 threshold (same formula as frozen_threshold, but on current ring)
    float live_thresh = (eng->baseline.count >= FV_MIN_BASELINE)
                        ? fv_percentile(eng->baseline.buf, eng->baseline.count, FV_PERCENTILE)
                        : 0.0f;

    // Decision metrics: pct_above, max_consec
    int above_count = 0;
    int max_consec = 0, c = 0;
    for (int i = 0; i < n_scores; i++) {
        if (scores[i] > 0.0f) {
            above_count++;
            if (++c > max_consec) max_consec = c;
        } else {
            c = 0;
        }
    }
    float pct_above = (n_scores > 0) ? (float)above_count / (float)n_scores : 0.0f;

    // Observation progress (0-100%)
    float obs_progress = 0.0f;
    if ((eng->state == FV_STATUS_OBSERVING || eng->state == FV_STATUS_WAITING)
            && eng->active_event.obs_end_time > eng->active_event.obs_start_time) {
        float now_s = (float)esp_timer_get_time() / 1e6f;
        float elapsed = now_s - eng->active_event.detection_time;
        float total_window = eng->active_event.obs_end_time - eng->active_event.detection_time;
        obs_progress = 100.0f * elapsed / total_window;
        if (obs_progress > 100.0f) obs_progress = 100.0f;
        if (obs_progress < 0.0f) obs_progress = 0.0f;
    }

    // Still ratio
    float still_ratio = (eng->active_event.total_obs > 0)
        ? (float)eng->active_event.still_count / (float)eng->active_event.total_obs
        : 0.0f;

    // Find max score
    float max_score = scores[0];
    for (int i = 1; i < n_scores; i++)
        if (scores[i] > max_score) max_score = scores[i];

    ptr += snprintf(msg + ptr, size_msg - ptr,
        "],\"motion_std\":%.6f,"
        "\"fall_rule\":%s,"
        "\"fall_reason\":\"%s\","
        "\"pct_above\":%.4f,"
        "\"max_consec\":%d,"
        "\"val_status\":\"%s\","
        "\"val_still\":%d,"
        "\"val_total\":%d,"
        "\"obs_progress\":%.1f,"
        "\"still_ratio\":%.2f,"
        "\"max_score\":%.4f,"
        "\"motion_thresh\":%.4f,"
        "\"dapd_us\":%lld,"
        "\"svm_us\":%lld}",
        motion_std,
        cr->fell ? "true" : "false",
        cr->reason,
        pct_above,
        max_consec,
        fv_status_str(eng->state),
        eng->active_event.still_count,
        eng->active_event.total_obs,
        obs_progress,
        still_ratio,
        max_score,
        live_thresh,
        dapd_us,
        svm_us);

    LOGI("DETAIL payload: %s", msg);
    ZC_TRY(zc_report_send, msg, strlen(msg));
    checked_free(msg);
}


// Fall alert payload, sent only on state transitions.
void send_fall_alert(const fv_engine_t *eng)
{
    char msg[256];
    float still_ratio = (eng->active_event.total_obs > 0)
        ? (float)eng->active_event.still_count / (float)eng->active_event.total_obs
        : 0.0f;

    snprintf(msg, sizeof(msg),
        "{\"fall_alert\":true,"
        "\"fall_status\":\"%s\","
        "\"fall_score\":%.4f,"
        "\"fall_reason\":\"%s\","
        "\"still_ratio\":%.2f}",
        fv_status_str(eng->state),
        eng->active_event.max_score,
        eng->active_event.trigger_reason ? eng->active_event.trigger_reason : "",
        still_ratio);

    LOGI("FALL_ALERT payload: %s", msg);
    ZC_TRY(zc_report_send, msg, strlen(msg));

    char supabase_msg[256];
    snprintf(supabase_msg, sizeof(supabase_msg), "{\"sensor\": \"%s\"}", zc_zf_rx_cfg.sensor_id);
    send_supabase(supabase_msg, "FALL_ALERT");
}
#endif /* FV_ENABLED */


/**
 * @brief Handler for the HTTP events to get the time
 *
 * @param evt Event
 * @return esp_err_t
 */
esp_err_t http_event_get_time_handler(esp_http_client_event_t *evt)
{
    static char *buffer = NULL;
    static int total_len = 0;

    switch (evt->event_id) {
    case HTTP_EVENT_ON_DATA:
        if (!esp_http_client_is_chunked_response(evt->client)) {
            if (buffer == NULL) {
                buffer = malloc(evt->data_len + 1);
                total_len = 0;
            } else {
                buffer = realloc(buffer, total_len + evt->data_len + 1);
            }
            memcpy(buffer + total_len, evt->data, evt->data_len);
            total_len += evt->data_len;
            buffer[total_len] = 0;
        }
        break;

    case HTTP_EVENT_ON_FINISH:
        if (buffer != NULL) {
            LOGI("Réponse JSON : %s",
                     buffer);
            int unixtime = extract_unixtime(buffer);
            if (unixtime != -1) {
                LOGI("Unixtime : %d",
                         unixtime);
                // Stockez ou utilisez la valeur ici
                set_system_time(unixtime);
            }
            free(buffer);
            buffer = NULL;
            total_len = 0;
        }
        break;

    case HTTP_EVENT_DISCONNECTED:
        if (buffer != NULL) {
            free(buffer);
            buffer = NULL;
            total_len = 0;
        }
        break;

    default:
        break;
    }
    return ESP_OK;
}


/**
 * @brief Send a HTTP GET request to get the time from the server
 *
 */
void http_get_time_request(void)
{
    esp_http_client_config_t config = {
        .url = "http://worldtimeapi.org/api/timezone/Europe/Paris",
        .event_handler = http_event_get_time_handler,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK) {
        LOGI(
                 "HTTP GET Status = %d, content_length = %lld",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        LOGE("Échec de la requête HTTP GET : %s",
                 esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}


/**
 * @brief Initialize the SNTP module to obtain the current time.
 *
 */
void initialize_sntp(void)
{
    LOGI("Initializing SNTP");
    esp_sntp_setservername(0, "pool.ntp.org");     // Serveur NTP
    esp_sntp_setservername(1, "time.google.com");  // Google's NTP Server
    esp_sntp_setservername(2, "time.windows.com"); // Backup NTP Server
    esp_sntp_setservername(3, "time.apple.com");

    esp_sntp_init();
}


/**
 * @brief Obtain the current time from the SNTP server.
 *
 */
void obtain_time(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 3;

    initialize_sntp();
    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        LOGI("Waiting for system time to be set... (%d/%d)",
                 retry, retry_count);
        // Feed WDT in 1 s chunks: a plain vTaskDelay(5000) without any reset
        // would exceed the 15 s (or 30 s) TWDT window if called from is_alive_task.
        for (int i = 0; i < 5; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset(); // no-op (ESP_ERR_NOT_FOUND) if caller is not subscribed
        }
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year > (2016 - 1900)) {
        LOGI("Time synchronized successfully");
        got_timestamp = true;
        return;
    }

    http_get_time_request();
    setenv("TZ", "UTC", 1);
    tzset();

    if (timeinfo.tm_year < (2016 - 1900)) {
        LOGE("Failed to synchronize time");
        got_timestamp = false;
    } else {
        LOGI("Time synchronized successfully");
        got_timestamp = true;
        return;
    }
}


/**
 * @brief Send isAlive information message to the server
 *
 * @param pvParameters
 */
void is_alive_task(void *pvParameters)
{
    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor = {
        .range_min = 20,
        .range_max = 100,
    };
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));
    int size_message = 256;
    char *message = (char *)checked_malloc(size_message);
    while (true) {
        esp_task_wdt_reset();

        csi_last_isAlive = csi_count - csi_last_isAlive;

        is_alive_counter++;

        float temp = get_chip_temperature(temp_handle);
        uint32_t free_ram = esp_get_free_heap_size();
        float cpu_load = calculate_cpu_load();
        wifi_ap_record_t ap_info;
        int8_t rssi = -1;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
        }

        // Build the JSON payload
        snprintf(
            message, size_message,
            "{\"temperature\":%.2f,\"ram_left\":%.2f,\"cpu_load\":%.2f,"
            "\"rssi\":%d,\"csi_since_last_is_alive\":%d, \"model\":\"SVM\"}",
            temp, (float)free_ram / 1000, cpu_load, rssi, csi_last_isAlive);

        if (csi_last_isAlive <= 3000)
            low_csi++;
        else
            low_csi = 0;
        if (low_csi >= 5) {
            LOGE("CSI is too low, RESTART");
            esp_restart();
        }

        csi_last_isAlive = csi_count;

        if (rssi != -1) {
            LOGI("isAlive payload: %s", message);
            ZC_TRY(zc_report_send, message, strlen(message));

            if (!got_timestamp) {
                obtain_time();
            }
        } else {
            LOGE("Failed to get AP info message: %s", message);
        }

        // Chunked sleep: feed WDT in RX_WDT_CHUNK_MS steps so report_status_delay
        // (potentially minutes) does not exceed the 15s WDT window.
        int total_ms = (int)(zc_zf_rx_cfg.report_status_delay * 1000);
        while (total_ms > 0) {
            int chunk = (total_ms < RX_WDT_CHUNK_MS) ? total_ms : RX_WDT_CHUNK_MS;
            vTaskDelay(pdMS_TO_TICKS(chunk));
            esp_task_wdt_reset();
            total_ms -= chunk;
        }
    }
}



/* ── CSI UART TX task ──────────────────────────────────────────────────────
   Serialises queued CSI packets into binary frames and writes to stdout.     
 
   Frame layout (137 bytes):                                                   
   [0xCA][0xFE][seq:u16le][ts_us:u32le][amp[64]:i16le x64][xor:u8]         
   
   Map frame (variable, sent once after subcarrier map is built):             
   [0xCA][0xFF][count:u8][idx[count]:u8][xor:u8]                            
   
   Sync bytes 0xCA/0xFE/0xFF are > 0x7F and never appear in ASCII ESP_LOGI   
   output, making text/binary demultiplexing safe on the Python side.         */
static void csi_uart_tx_task(void *arg)
{
    csi_uart_pkt_t pkt;
    char csi_data[64 * 8];

    while (1) {
        // Wait packets from wifi callback
        if (xQueueReceive(s_csi_uart_q, &pkt, portMAX_DELAY) == pdTRUE) {
            
            // Format samples in a csv string
            int csi_len = snprintf(csi_data, sizeof(csi_data), "%d", pkt.amp[0]);
            for (int i = 1; i < CSI_COLS_SIZE; i++) {
                csi_len += snprintf(csi_data + csi_len, sizeof(csi_data) - csi_len, ",%d", pkt.amp[i]);
            }

            // Safe print with mutex to avoid interference with logs
            if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);
            printf("AMP:%u:%s\n", pkt.seq, csi_data);
            fflush(stdout);
            if (stdout_mutex) xSemaphoreGive(stdout_mutex);
        }
    }
}


void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        return;
    }

    if (memcmp(info->mac, ctx, 6)) {
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;

    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);

    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);

    // Construct the CSI data as a comma-separated string

    // * "Raw" CSI, what we used before
    // char csi_data[128 * 8];  // Max size of CSI data
    // int csi_len = snprintf(csi_data, sizeof(csi_data), "%d", info->buf[0]);
    // for (int i = 1; i < info->len; i++) {
    //     csi_len += snprintf(csi_data + csi_len, sizeof(csi_data) - csi_len, ",%d", info->buf[i]);
    // }
    
    // printf("CSI;%d;%s;End\nGAIN_COR;%f;End", rx_ctrl->timestamp, csi_data, compensate_gain);
    last_ts = rx_ctrl->timestamp;

    // Calculate the CSI absolute values
    signed short int abs_csi_sample[64];
    float CSI_r = 0;
    float CSI_i = 0;

    for (int i = 0; i < info->len; i++) {
        if (i % 2 == 0) {
            // CSI_r = compensate_gain * info->buf[i];
            CSI_r = info->buf[i];
        } else if (i % 2 == 1) {
            // CSI_i = compensate_gain * info->buf[i];
            CSI_i = info->buf[i];
            // abs_csi_sample[i / 2] = (signed short int)(sqrt(
            //     CSI_r * CSI_r + CSI_i * CSI_i) * 100);
            abs_csi_sample[i / 2] = (signed short int)(sqrtf(
                CSI_r * CSI_r + CSI_i * CSI_i));
        }
    }

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);

    // fill csi cache with info buff data (protected by mutex for concurrent readers)
    if (csi_mutex) xSemaphoreTake(csi_mutex, portMAX_DELAY);

    memcpy(&csi_cache[row_now * CSI_COLS_SIZE], abs_csi_sample, CSI_COLS_SIZE * sizeof(short int));
    row_now = (row_now + 1) % CSI_CACHE_SIZE;
    packet_counter += 1;
    csi_count++;
    if (csi_count == SIZE_MAX) {
        csi_count = CSI_CACHE_SIZE;
        LOGI("csi_count overflow\n");
    }

    uint16_t this_seq = s_csi_seq++;   // Increment inside mutex so dapd_calc_task sees consistent seq
    if (csi_mutex) xSemaphoreGive(csi_mutex);

    // Enqueue raw CSI for UART streaming (non-blocking: drop if full)
    if (s_csi_uart_q) {
        csi_uart_pkt_t p;
        p.seq    = this_seq;
        p.ts_us  = (uint32_t)rx_ctrl->timestamp;
        memcpy(p.amp, abs_csi_sample, CSI_COLS_SIZE * sizeof(int16_t));
        xQueueSend(s_csi_uart_q, &p, 0);
    }

}


float stdev(float *csi, int size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += csi[i];
    }
    float mean = sum / (float)size;
    float variance_sum = 0.0f;
    for (int i = 0; i < size; i++) {
        float d = csi[i] - mean;
        variance_sum += d * d;
    }
    float variance = variance_sum / (float)(size - 1);
    return sqrtf(variance);
}

// DAPD batch -> SVM inference task
//
void dapd_calc_task(void *pvParameters) {
    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);
    const int T  = CSI_WINDOW_SIZE;  // 400
    const int n_bins = 17;           // from limite_seuil=0.056

    //  Allocate only active buffers in PSRAM
    // Fused pipeline eliminates work_buf and reduced_buf
    float *csi_float  = heap_caps_malloc(T * DAPD_M_ACTIVE_MAX * sizeof(float), MALLOC_CAP_SPIRAM);
    float *smk_batch  = heap_caps_malloc(SVM_SLIDING_K_TOTAL * 4 * DAPD_M_ACTIVE_MAX * DAPD_N_BINS * sizeof(float), MALLOC_CAP_SPIRAM);
    float *masked_buf  = heap_caps_malloc(SVM_N_FEATURES * sizeof(float), MALLOC_CAP_SPIRAM);
    float *inv_scale   = heap_caps_malloc(SVM_N_FEATURES * sizeof(float), MALLOC_CAP_SPIRAM);
    float *exp_bins    = heap_caps_malloc((DAPD_N_BINS + 1) * sizeof(float), MALLOC_CAP_SPIRAM);
    float *scores      = heap_caps_malloc(SVM_SLIDING_N_INFER * sizeof(float), MALLOC_CAP_SPIRAM);

    if (!csi_float || !smk_batch || !masked_buf || !inv_scale || !exp_bins || !scores) {
        LOGE("Failed to allocate PSRAM buffers");
        heap_caps_free(csi_float);
        heap_caps_free(smk_batch);
        heap_caps_free(masked_buf);
        heap_caps_free(inv_scale);
        heap_caps_free(exp_bins);
        heap_caps_free(scores);
        vTaskDelete(NULL);
        return;
    }

    // Pre-compute inverse scale once
    svm_precompute_inv_scale(inv_scale);

    bool initialized = false;
    int M = 0;
    dapd_subcarrier_map_t map;
    dapd_params_t params;

#if FV_ENABLED
    fv_engine_t *fv_eng = (fv_engine_t *)heap_caps_malloc(sizeof(fv_engine_t), MALLOC_CAP_SPIRAM);
    if (!fv_eng) {
        LOGE("Failed to allocate fv_engine_t in PSRAM");
    } else {
        fv_engine_init(fv_eng);
        LOGI("Fall validation engine initialized "
                 "(t_wait=%.1fs t_obs=%.1fs f_still=%.2f P%d)",
                 FV_T_WAIT, FV_T_OBS, FV_F_STILL, FV_PERCENTILE);
    }
#endif

    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (true) {
        esp_task_wdt_reset();

        // Pause DAPD processing during OTA download
        if (ota_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int current_row = -1;

        // Check if we have received the required step of new packets
        uint16_t batch_seq_end = 0;  // seq of last sample used by batch
        xSemaphoreTake(csi_mutex, portMAX_DELAY);
        if (packet_counter >= SLIDING_STEP) {
            current_row = row_now;
            packet_counter = 0;
            batch_seq_end = (uint16_t)(s_csi_seq - 1);
        }
        xSemaphoreGive(csi_mutex);

        if (current_row != -1) {
            int64_t start = esp_timer_get_time();
            int start_row = (current_row - T + CSI_CACHE_SIZE) % CSI_CACHE_SIZE;

            if (!initialized) {
                // Build the map of active subcarriers (once)
                // Hold mutex while scanning csi_cache to prevent race with wifi_csi_rx_cb
                xSemaphoreTake(csi_mutex, portMAX_DELAY);
                map.count = 0;
                for (int m = 0; m < CSI_COLS_SIZE && map.count < DAPD_M_ACTIVE_MAX; m++) {
                    bool nonzero = false;
                    for (int t = 0; t < T; t++) {
                        int phys = (start_row + t) % CSI_CACHE_SIZE;
                        if (csi_cache[phys * CSI_COLS_SIZE + m] != 0) {
                            nonzero = true; break;
                        }
                    }
                    if (nonzero) map.active[map.count++] = m;
                }
                xSemaphoreGive(csi_mutex);
                M = map.count;

                // The SVM mask was built for exactly SVM_DAPD_M=52 active subcarriers.
                if (M != SVM_DAPD_M) {
                    LOGE("Active subcarriers M=%d != SVM_DAPD_M=%d - SVM predictions will be wrong!", M, SVM_DAPD_M);
                }

                dapd_params_default(&params);
                params.train_ratio = (float)(CSI_WINDOW_SIZE - 65) / (float)CSI_WINDOW_SIZE;
                params.delta = 25;

                // Precompute exponential bins once (depend only on fixed params)
                int nb = (int)(1.0f / params.limite_seuil);
                float exp_alpha = expf(params.alpha);
                for (int i = 0; i <= nb; i++) {
                    float t = (float)i / (float)nb;
                    exp_bins[i] = params.min_seuil
                        + (1.0f - params.min_seuil)
                        * (expf(params.alpha * t) - 1.0f) / (exp_alpha - 1.0f);
                }

                initialized = true;

                {
                    char map_buf[8 + DAPD_M_ACTIVE_MAX * 4]; // "MAP:64:" + max "63," * 64
                    int map_len = snprintf(map_buf, sizeof(map_buf), "MAP:%d:", map.count);
                    for (int i = 0; i < map.count; i++)
                        map_len += snprintf(map_buf + map_len, sizeof(map_buf) - map_len,
                                            "%d%s", map.active[i], (i == map.count - 1) ? "" : ",");
                    if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);
                    printf("%s\n", map_buf);
                    fflush(stdout);
                    if (stdout_mutex) xSemaphoreGive(stdout_mutex);
                }
            }

            // Thread-Safe Data Extraction from Cache
            // Hold mutex during copy to prevent race with wifi_csi_rx_cb writing to csi_cache.
            // Copy takes ~2ms for 800×52 entries; at 100 Hz callback period (10ms) this is safe.
            xSemaphoreTake(csi_mutex, portMAX_DELAY);
            float max_amp_f = 0.0f;
            for (int i = 0; i < T; i++) {
                int phys = (start_row + i) % CSI_CACHE_SIZE;
                const int16_t *row = &csi_cache[phys * CSI_COLS_SIZE];
                float *dst = &csi_float[i * M];
                for (int j = 0; j < M; j++) {
                    dst[j] = (float)row[map.active[j]];
                    float abs_v = fabsf(dst[j]);
                    if (abs_v > max_amp_f) max_amp_f = abs_v;
                }
            }
            xSemaphoreGive(csi_mutex);
            params.sigma = (int)max_amp_f + 1;
            if (params.sigma < 1) params.sigma = 1;

            // Run DAPD Batch
            int Td = (int)(T * params.train_ratio); 
            int Tw = T - Td;                        
            int K  = Td / params.delta;             
            int K_out = 0, n_bins_out = 0;

            // Run DAPD standard (ground truth)
            dapd_params_t params_std = params; params_std.use_fast_exp = 0;
            int64_t t0_std = esp_timer_get_time();
#if DAPD_PROFILE
            dapd_timing_t dapd_timing_std = {0};
            dapd_batch(csi_float, T, M, Td, Tw, K, &params_std, exp_bins, smk_batch, &K_out, &n_bins_out, &dapd_timing_std);
#else
            dapd_batch(csi_float, T, M, Td, Tw, K, &params_std, exp_bins, smk_batch, &K_out, &n_bins_out);
#endif
            int64_t dapd_std_time = esp_timer_get_time() - t0_std;

            // --- DIAG_LT computation (always needed for motion indicator) ---
            float motion_std = 0.0f;
            {
                int K4 = K_out * 4;
                int moitie = n_bins_out / 2;

#if DAPD_DIAG_METRICS
                if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);

                /* DIAG_ZF: zero fraction per fT slice - shape (K4,)
                 * Format: DIAG_ZF:<K4>:<M>:<v0>,<v1>,...,<v_{K4-1}>\n */
                printf("DIAG_ZF:%d:%d:", K4, M);
                for (int fT = 0; fT < K4; fT++) {
                    int zeros = 0;
                    for (int m = 0; m < M; m++)
                        for (int b = 0; b < n_bins_out; b++)
                            if (smk_batch[(fT * M + m) * n_bins_out + b] == 0.0f)
                                zeros++;
                    float zf = (float)zeros / (M * n_bins_out);
                    printf("%.6f%s", zf, (fT < K4 - 1) ? "," : "\n");
                }

                /* DIAG_LT header */
                printf("DIAG_LT:%d:%d:", K4, M);
#endif /* DAPD_DIAG_METRICS */

                /* Compute LT values + accumulate sum/sum_sq for motion indicator.
                 * LT = index of first non-zero bin in first half, per (fT, m).
                 * motion_std = population std of entire LT matrix (matches np.std). */
                int total_lt = K4 * M;
                int lt_idx = 0;
                int lt_sum = 0, lt_sum_sq = 0;

                for (int fT = 0; fT < K4; fT++) {
                    for (int m = 0; m < M; m++) {
                        int first = moitie;
                        for (int f2 = 0; f2 < moitie; f2++) {
                            if (smk_batch[(fT * M + m) * n_bins_out + f2] > 0.0f) {
                                first = f2;
                                break;
                            }
                        }
#if DAPD_DIAG_METRICS
                        printf("%d%s", first, (lt_idx < total_lt - 1) ? "," : "\n");
#endif
                        lt_sum += first;
                        lt_sum_sq += first * first;
                        lt_idx++;
                    }
                }

#if DAPD_DIAG_METRICS
                if (stdout_mutex) xSemaphoreGive(stdout_mutex);
#endif

                /* Compute motion_std: population std (ddof=0) to match np.std() */
                if (total_lt > 0) {
                    float lt_mean = (float)lt_sum / (float)total_lt;
                    float lt_var  = (float)lt_sum_sq / (float)total_lt - lt_mean * lt_mean;
                    motion_std = sqrtf(lt_var > 0.0f ? lt_var : 0.0f);
                }
            }

            // --- Sliding SVM inference (N_INFER=1 when EXTENDED_WINDOW=0, =17 when =1) ---
            int64_t svm_start = esp_timer_get_time();
            int stride_per_k = 4 * M * n_bins_out;

            for (int si = 0; si < SVM_SLIDING_N_INFER; si++) {
                scores[si] = svm_fused_pipeline(
                    &smk_batch[si * stride_per_k],
                    M, n_bins_out, masked_buf, inv_scale);
            }
            int64_t svm_time = esp_timer_get_time() - svm_start;
            int64_t total_time = esp_timer_get_time() - start;
            esp_task_wdt_reset();  // SVM inference can take ~2s; reset after it completes

            // Validate OTA after first successful SVM inference
            static bool ota_validated = false;
            if (!ota_validated) {
                ota_mark_valid();
                ota_validated = true;
            }

            // --- Output ---
#if OUTPUT_MODE == OUTPUT_UART || OUTPUT_MODE == OUTPUT_BOTH
            // Backward-compatible line for existing Python tools (uses scores[0])
            int pred_compat = predict_svm_rbf(scores[0], SVM_THRESHOLD);
            LOGI("score=%.4f pred=%d seq_end=%u "
                     "(DAPD: %lld us, SVM: %lld us, total: %lld us)",
                     scores[0], pred_compat, (unsigned)batch_seq_end,
                     dapd_std_time, svm_time, total_time);

            // Multi-score line (only when sliding window is active)
            if (SVM_SLIDING_N_INFER > 1) {
                if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);
                printf("SVM_MULTI:[");
                for (int si = 0; si < SVM_SLIDING_N_INFER; si++)
                    printf("%.4f%s", scores[si], si < SVM_SLIDING_N_INFER - 1 ? "," : "");
                printf("]\n");
                fflush(stdout);
                if (stdout_mutex) xSemaphoreGive(stdout_mutex);
            }
#endif

#if FV_ENABLED
            // Fall validation engine
            if (fv_eng) {
                float now_s = (float)esp_timer_get_time() / 1e6f;

                // Always push motion baseline (even when no fall)
                fv_baseline_push(fv_eng, motion_std);

                // Compound fall rule on all sliding-window scores
                fv_compound_result_t cr = fv_compound_fall_rule(
                    scores, SVM_SLIDING_N_INFER);

                // Find max score for event record
                float max_score = scores[0];
                for (int i = 1; i < SVM_SLIDING_N_INFER; i++)
                    if (scores[i] > max_score) max_score = scores[i];

                // Advance state machine
                fv_status_t prev_state = fv_eng->state;
                fv_engine_update(fv_eng, now_s, cr.fell, cr.reason,
                                 max_score, motion_std);

#if OUTPUT_MODE == OUTPUT_UART || OUTPUT_MODE == OUTPUT_BOTH
                // UART FV log lines (for side-by-side comparison with Python)
                LOGI("motion=%.6f state=%s still=%d/%d thr=%.6f",
                         motion_std, fv_status_str(fv_eng->state),
                         fv_eng->active_event.still_count,
                         fv_eng->active_event.total_obs,
                         fv_eng->frozen_threshold);
                if (cr.fell)
                    LOGW("Compound rule: %s (max=%.4f)",
                             cr.reason, max_score);
                if (fv_eng->state != prev_state)
                    LOGI("Transition: %s -> %s",
                             fv_status_str(prev_state),
                             fv_status_str(fv_eng->state));
#endif

#if OUTPUT_MODE == OUTPUT_TB || OUTPUT_MODE == OUTPUT_BOTH
                // Detailed telemetry (replaces send_svm_scores)
                send_validation_telemetry(scores, SVM_SLIDING_N_INFER,
                                          motion_std, fv_eng, &cr,
                                          dapd_std_time, svm_time);

                // Fall alert on state transitions only
                if (fv_eng->state != prev_state
                        && (fv_eng->state == FV_STATUS_WAITING
                         || fv_eng->state == FV_STATUS_VALIDATED
                         || fv_eng->state == FV_STATUS_REJECTED)) {
                    send_fall_alert(fv_eng);
                }
#endif
            }
#else /* FV_ENABLED == 0 */
#if OUTPUT_MODE == OUTPUT_TB || OUTPUT_MODE == OUTPUT_BOTH
            send_svm_scores(scores, SVM_SLIDING_N_INFER, dapd_std_time, svm_time);
#endif
#endif /* FV_ENABLED */
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        LOGE("Disconnected from Wi-Fi, attempting to reconnect...");
        led_flag = false;
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1-second delay
        esp_wifi_connect();  // Reattempt connection
    }
    else if (event_id == WIFI_EVENT_STA_STOP) {
        //handle_error("wifi_event_handler", "Wi-Fi stopped.");
        //LOGW("Wi-Fi stopped.");
        // Optional: Handle additional cleanup here if necessary
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        LOGI("Connected to Wi-Fi, IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        atomic_store(&got_ip, 1);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void wifi_csi_init()
{
    /**
     * @brief In order to ensure the compatibility of routers, only LLTF sub-carriers are selected.
     */
    // Channel state information (CSI) configuration type
    wifi_csi_config_t csi_config = {
        .lltf_en = true,           // enable to receive LLTF data
        .htltf_en = false,         // HT long training field data
        .stbc_htltf2_en = false,   // space time block code HT LTF data
        .ltf_merge_en = true,      // enable to generate HTLFT data by averagin LLTF and HTLTF when receiving HT paquets
        .channel_filter_en = true, // enable to turn on channel filter to smooth adjacent sub-carrier; disable it to keep independence of adjacent sub-carrier
        .manu_scale = true,        // manually scale the CSI data by left shifting or automatically scale the CSI data
        .shift = true,
    };

    static wifi_ap_record_t s_ap_info = {0};
    // Access Point information
    esp_wifi_sta_get_ap_info(&s_ap_info);                    // get info of AP
    esp_wifi_set_csi_config(&csi_config);                    // set CSI data configuration
    esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid); // register the RX callback function of CSI data
    esp_wifi_set_csi(true);                                  // enable CSI
    esp_wifi_set_bandwidth(ESP_IF_WIFI_STA ,WIFI_BW_HT40);   // set the bandwidth of the Wi-Fi connection
}


void wifi_connect(void)
{
    // Create Wi-Fi event group
    wifi_event_group = xEventGroupCreate();

    // Initialize the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create a default Wi-Fi station interface
    esp_netif_t *wifi_sta = esp_netif_create_default_wifi_sta();

    #if !WIFI_USE_DHCP
        // Static IP: direct router only (no mobile hotspot)
        esp_netif_ip_info_t ip_info;
        IP4_ADDR(&ip_info.ip, 192, 168, 4, WIFI_STATIC_IP_LAST_OCTET);
        IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
        esp_netif_dns_info_t dns_info;
        IP4_ADDR(&dns_info.ip.u_addr.ip4, 192, 168, 4, 1);
        dns_info.ip.type = 0U;
        ESP_ERROR_CHECK(esp_netif_dhcpc_stop(wifi_sta));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(wifi_sta, &ip_info));
        ESP_ERROR_CHECK(esp_netif_set_dns_info(wifi_sta, ESP_NETIF_DNS_MAIN, &dns_info));
    #endif
        // WIFI_USE_DHCP=1: DHCP active by default on create_default_wifi_sta()
        // TX propagates carrier DNS (e.g. 10.106.108.250) → works through hotspot NAT

    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handler for Wi-Fi and IP events
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // Set Wi-Fi configuration
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strncpy((char *)wifi_config.sta.ssid,     zc_zf_rx_cfg.wifi_ap_ssid,     sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password,  zc_zf_rx_cfg.wifi_ap_password, sizeof(wifi_config.sta.password) - 1);

    // Set Wi-Fi mode to station (STA)
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    // Wait until connected
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    LOGI("Wi-Fi connected successfully");
}


void print_wifi_signal_strength(void *pvParameter)
{
    while (1) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            // printf("RSSI2;%d;0;End\n", ap_info.rssi);
            rssi = ap_info.rssi;
        } else {
            printf("Failed to get Wi-Fi signal strength");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}


void app_main() {
    stdout_mutex = xSemaphoreCreateMutex();
    esp_log_set_vprintf(locked_vprintf);

    if (stdout_mutex) xSemaphoreTake(stdout_mutex, portMAX_DELAY);
    printf("\n");
    fflush(stdout);
    if (stdout_mutex) xSemaphoreGive(stdout_mutex);

    LOGI("App Version: %d.%d.%d-RX (Git: %s)",
            APP_VER_MAJOR, APP_VER_MINOR, APP_VER_PATCH, APP_VER_GIT_HASH);
    printf("\n");

    // PSRAM allocation
    csi_cache = (short int *)heap_caps_malloc(CSI_COLS_SIZE * CSI_CACHE_SIZE * sizeof(short int), MALLOC_CAP_SPIRAM);
    if (csi_cache == NULL) {
        LOGE("Failed to allocate csi_cache in PSRAM!");
        esp_restart();
    }
    memset(csi_cache, 0, CSI_COLS_SIZE * CSI_CACHE_SIZE * sizeof(short int));

    esp_log_level_set("*", ESP_LOG_NONE);
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    ota_check_partition_layout();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_log_level_set("*", ESP_LOG_INFO);

    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);
    zc_zf_rx_err_init();

    zc_logs_cfg.enable         = true;
    zc_logs_cfg.enable_error   = true;
    zc_logs_cfg.enable_warning = true;
    zc_logs_cfg.enable_info    = true;
    zc_logs_cfg.enable_debug   = false;
    zc_logs_cfg.enable_verbose = false;

    zc_zf_rx_cfg_load();
    zc_zf_rx_cfg_print();
    
    gpio_set_direction(LED_STATUS_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_STATUS_PIN);

    LOGW("Before init wifi");

    // Disable automatic \r injection to make stdout binary-safe for ANY console type
    esp_vfs_dev_uart_port_set_tx_line_endings(0, ESP_LINE_ENDINGS_LF);
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
#endif
#if CONFIG_ESP_CONSOLE_USB_CDC
    esp_vfs_dev_cdcacm_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
#endif

    wifi_connect();

    config.url = zc_zf_rx_cfg.tb_url;
    config.method = HTTP_METHOD_POST;
    config.cert_pem = (const char *)ZC_CA_CERT_PEM_START;
    config.timeout_ms = 5000;

    zc_report_init();
    xTaskCreate(zc_report_send_task, "report_send", 8192, &config, 4, NULL);

    config_supabase.url = zc_zf_rx_cfg.supabase_alert_url;
    config_supabase.method = HTTP_METHOD_POST;
    config_supabase.cert_pem = (const char *)ZC_SUPA_CERT_PEM_START;
    config_supabase.timeout_ms = 5000;
    config_supabase.buffer_size_tx = 1024;
    client_supabase = esp_http_client_init(&config_supabase);

    wifi_csi_init();

    // Start OTA check task and safety watchdog
    ota_start_task();

    char ver_msg[96];
    snprintf(ver_msg, sizeof(ver_msg),
             "{\"firmware_version\":\"%s\",\"role\":\"RX\"}",
             esp_app_get_description()->version);
    LOGI("Startup telemetry: %s", ver_msg);
    ZC_TRY(zc_report_send, ver_msg, strlen(ver_msg));

    // One-shot abnormal reset telemetry: logged every boot after a crash/WDT/brownout.
    esp_reset_reason_t rr = esp_reset_reason();
    if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT ||
        rr == ESP_RST_PANIC    || rr == ESP_RST_BROWNOUT) {
        const char *rr_name = (rr == ESP_RST_TASK_WDT) ? "task_wdt" :
                              (rr == ESP_RST_INT_WDT)  ? "int_wdt"  :
                              (rr == ESP_RST_PANIC)     ? "panic"    : "brownout";
        char rr_msg[128];
        snprintf(rr_msg, sizeof(rr_msg),
                 "{\"event\":\"abnormal_reset\",\"reason\":\"%s\",\"role\":\"RX\"}", rr_name);
        LOGW("Abnormal reset detected: %s", rr_name);
        ZC_TRY(zc_report_send, rr_msg, strlen(rr_msg));
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    csi_mutex = xSemaphoreCreateMutex();
    if (!csi_mutex) {
        LOGE("Failed to create csi_mutex");
        esp_restart();
    }

#if CSI_RAW_STREAM
    /* Create UART TX queue and streaming task before WiFi starts.
     * Enabled by CSI_RAW_STREAM=1 in CMakeLists.txt.
     * Read on PC with: python tools/dapd_realtime.py --port <PORT> */
    s_csi_uart_q = xQueueCreate(100, sizeof(csi_uart_pkt_t));
    if (!s_csi_uart_q) {
        LOGE("Failed to create CSI UART queue");
        esp_restart();
    }

    // Set stack size to 4096 to prevent stack overflow
    xTaskCreate(csi_uart_tx_task, "csi_uart_tx", 4096, NULL, 5, NULL);
#endif /* CSI_RAW_STREAM */

    LOGW("Creating Task");
    // xTaskCreate(&calc_task, "calc_task", 8192, NULL, 8, NULL);
    xTaskCreate(&dapd_calc_task, "dapd_task", 16384, NULL, 8, NULL); // needs more stack for GKDE
    xTaskCreate(is_alive_task, "is_alive_task", 8192, NULL, 5, NULL);
    // -- DAPD test on a single-subcarrier signal (data from MTF test) --
    // static const float signal[] = {1.412, 1.458, 1.417, 1.43, 1.474, 1.432, 1.361, 1.264, 1.281, 1.347, 1.423, 1.634, 1.624, 1.575, 1.505, 1.391, 1.367, 1.378, 1.403, 1.446, 1.447, 1.415, 1.343, 1.356, 1.458, 1.416, 1.365, 1.333, 1.445, 1.438, 1.378, 1.362, 1.304, 1.239, 1.177, 1.159, 1.312, 1.399, 1.454, 1.381, 1.379, 1.4, 1.263, 1.265, 1.263, 1.22, 1.271, 1.369, 1.449, 1.383, 1.376, 1.38, 1.396, 1.391, 1.343, 1.502, 1.327, 1.316, 1.406, 1.509, 1.419, 1.458, 1.384, 1.392, 1.433, 1.55, 1.487, 1.572, 1.567, 1.583, 1.477, 1.218, 1.268, 1.411, 1.452, 1.483, 1.491, 1.439, 1.382, 1.355, 1.301, 1.408, 1.439, 1.409, 1.405, 1.388, 1.399, 1.428, 1.395, 1.712, 1.408, 1.306, 1.334, 1.274, 1.354, 1.414, 1.634, 1.621, 1.817, 1.598, 1.51, 1.506, 1.414, 1.301, 1.211, 1.245, 1.25, 1.326, 1.309, 1.32, 1.288, 1.469, 1.471, 1.568, 1.471, 1.303, 1.26, 1.479, 1.593, 1.328, 1.404, 1.342, 1.351, 1.339, 1.456, 1.629, 1.605, 1.424, 1.556, 1.487, 1.602, 1.604, 1.592, 1.57, 1.533, 1.592, 1.56, 1.656, 1.527, 1.658, 1.461, 1.509, 1.391, 1.314, 1.311, 1.429, 1.525, 1.491, 1.367, 1.437, 1.472, 1.522, 1.389, 1.403, 1.405, 1.539, 1.523, 1.634, 1.599, 1.703, 1.532, 1.771, 1.719, 1.647, 1.729, 1.67, 1.579, 1.654, 1.375, 1.541, 1.534, 1.437, 1.484, 1.477, 1.474, 1.421, 1.567, 1.487, 1.382, 1.28, 1.282, 1.401, 1.354, 1.278, 1.51, 1.542, 1.362, 1.257, 1.294, 1.347, 1.315, 1.268, 1.305, 1.418, 1.385, 1.319, 1.132, 1.031, 1.035, 1.095, 1.185, 1.293, 1.378, 1.39, 1.516, 1.542, 1.582, 1.597, 1.507, 1.572, 1.539, 1.358, 1.351, 1.2, 1.166, 1.178, 1.256, 1.24, 1.226, 1.169, 1.102, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 1.213, 0.853, 0.923, 1.252, 1.54, 1.281, 1.634, 1.81, 1.704, 1.646, 1.971, 1.909, 1.836, 1.738, 1.658, 1.805, 1.714, 1.908, 1.757, 1.673, 1.929, 2.072, 2.05, 2.003, 2.073, 1.755, 1.755, 1.652, 1.482, 1.525, 1.497, 1.537, 1.746, 1.962, 1.991, 1.893, 1.781, 2.033, 2.016, 2.097, 1.838, 1.693, 1.803, 1.919, 1.911};
    // const int signal_len = sizeof(signal) / sizeof(signal[0]);

    {
        // // DAPD: no PCA needed, features are extracted directly from amplitude PDF
        // dapd_params_t params;
        // dapd_params_default(&params);
        // params.sigma = 3; // ceil(max(signal)) + 1
        // // params.num_points = 200;

        // int M = 1; // single subcarrier for this test
        // int n_bins = (int)(1.0f / params.limite_seuil); // 10
        // float smk_out[4 * 1 * DAPD_N_BINS]; // 4 distributions x 1 subcarrier x 10 bins

        // LOGW("Computing DAPD on test signal (%d samples, M=%d)...", signal_len, M);
        // int64_t t0 = esp_timer_get_time();

        // dapd_compute_window(signal, signal_len, &params, M, smk_out);

        // int64_t t1 = esp_timer_get_time();
        // LOGI("Done in %lld us. Output: 4 x %d x %d = %d features",
        //          t1 - t0, M, n_bins, 4 * M * n_bins);

        // const char *sm_names[] = {"Sm1", "Sm2", "Sm3", "Sm4"};
        // for (int s = 0; s < 4; s++) {
        //     printf("%s: ", sm_names[s]);
        //     for (int b = 0; b < n_bins; b++) {
        //         printf("%.4f ", smk_out[(s * M + 0) * n_bins + b]);
        //     }
        //     printf("\n");
        // }
    }

    while (1) {
        // printf("%d\n", led_flag);
        if (led_flag) {
            gpio_set_level(LED_STATUS_PIN, 1);
        } else {
            gpio_set_level(LED_STATUS_PIN, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

}