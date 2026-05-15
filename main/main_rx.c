#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_timer.h"

#define WIFI_SSID "CSI_PAIR_RX"
#define WIFI_PASS "password123"

static const char *TAG = "csi_rx";

// Stub implementations for gain control helpers
// Replace these with your actual component implementations if available
static esp_err_t esp_csi_gain_ctrl_get_rx_gain(const wifi_pkt_rx_ctrl_t *rx_ctrl, uint8_t *agc_gain, int8_t *fft_gain)
{
    if (!rx_ctrl || !agc_gain || !fft_gain) return ESP_ERR_INVALID_ARG;
    // Extract gain from rx_ctrl if available, otherwise use defaults
    *agc_gain = (rx_ctrl->rssi >= -50) ? 50 : (100 + rx_ctrl->rssi);  // Rough approximation
    *fft_gain = -5;  // Default FFT gain
    return ESP_OK;
}

static void esp_csi_gain_ctrl_record_rx_gain(uint8_t agc_gain, int8_t fft_gain)
{
    // TODO: Implement actual gain recording if needed
    // This could store gains in NVS, SD card, or just log them
}

static esp_err_t esp_csi_gain_ctrl_get_gain_compensation(int16_t *compensate_gain, uint8_t agc_gain, int8_t fft_gain)
{
    if (!compensate_gain) return ESP_ERR_INVALID_ARG;
    // Simple compensation calculation: AGC + FFT*10
    *compensate_gain = (int16_t)(agc_gain + fft_gain * 10);
    return ESP_OK;
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *csi_info)
{
    if (!csi_info) return;

    uint8_t agc_gain = 0;
    int8_t fft_gain = 0;
    int16_t compensate_gain = 0;

    esp_err_t ret = esp_csi_gain_ctrl_get_rx_gain(&csi_info->rx_ctrl, &agc_gain, &fft_gain);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to get rx gain: %d", ret);
    }

    esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);

    ret = esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to get compensation: %d", ret);
    }

    int len = csi_info->len;

    // Log summary to serial (UART0)
    ESP_LOGI(TAG, "CSI: len=%d mac=%02x:%02x:%02x:%02x:%02x:%02x rssi=%d agc=%u fft=%d comp=%d",
             len,
             csi_info->mac[0], csi_info->mac[1], csi_info->mac[2], csi_info->mac[3], csi_info->mac[4], csi_info->mac[5],
             csi_info->rx_ctrl.rssi, agc_gain, fft_gain, compensate_gain);

    // Print a CSV-style CSI_META line compatible with the Python parser.
    // Format: CSI_META,<rx_tag>,<rx_mac>,<time_us>,<wifi_time_us>,<probe_seq>,<tx_tag>,<src_tag>,<src_mac>,<dst_mac>,<rssi>,...,<rx_seq>,<csi_len>,<raw samples...>
    int to_print = len;
    int64_t time_us = esp_timer_get_time();
    // rx_tag - use a simple tag
    const char *rx_tag = "RX";
    // prepare MAC string
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             csi_info->mac[0], csi_info->mac[1], csi_info->mac[2], csi_info->mac[3], csi_info->mac[4], csi_info->mac[5]);

    // We don't have wifi_time_us/probe_seq/src/dst tags here — leave empty or zero
    int wifi_time_us = 0;
    int probe_seq = 0;
    const char *tx_tag = "";
    const char *src_tag = "";
    const char *src_mac = mac_str;
    const char *dst_mac = "";
    int rssi = csi_info->rx_ctrl.rssi;
    int rx_seq = 0;

    // Print header fields
    printf("CSI_META,%s,%s,%lld,%d,%d,%s,%s,%s,%s,%d,,,,%d,%d",
           rx_tag, mac_str, (long long)time_us, wifi_time_us, probe_seq,
           tx_tag, src_tag, src_mac, dst_mac, rssi, rx_seq, to_print);

    // Print raw samples as decimal integers
    for (int i = 0; i < to_print; ++i) {
        printf(" %d", (int)csi_info->buf[i]);
    }
    printf("\n");
}

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    esp_netif_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Configure CSI
    wifi_csi_config_t csi_config = {
        .lltf_en = 1,
        .htltf_en = 1,
        .stbc_htltf2_en = 1,
        .ltf_merge_en = 1,
        .channel_filter_en = 1,
        .manu_scale = 0,
        .shift = 0,
    };
    esp_wifi_set_csi_config(&csi_config);
    esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL);
    esp_wifi_set_csi(1);

    ESP_LOGI(TAG, "CSI RX started on SSID: %s", WIFI_SSID);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
