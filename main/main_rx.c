#include <math.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_csi_gain_ctrl.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#define TX_AP_SSID "CSI_TX_AP"
#define TX_AP_PASS "csi123456"

/* The RX uses its STA radio to connect to the TX AP for CSI.
 * A second simultaneous upstream AP is not possible with this simple STA setup;
 * send non-serial data through the TX AP network, or enable TX upstream APSTA. */
#define WIFI_SSID TX_AP_SSID
#define WIFI_PASS TX_AP_PASS
#define RX_PORT 12345
#define CSI_QUEUE_LEN 128
#define CSI_AMP_MAX_SUBCARRIERS 128
#define CSI_PRINT_SUBCARRIERS 64
#define UDP_RATE_LOG_ENABLED 1
#define CSI_RATE_LOG_ENABLED 1
#define CSI_FORMAT_LOG_ENABLED 1
#define RX_GAIN_BASELINE_PACKETS 100

static const char *TAG = "csi_rx";

static bool wifi_connected = false;
static QueueHandle_t csi_queue = NULL;
static uint8_t ap_bssid[6] = {0};
static bool ap_bssid_valid = false;
static volatile uint32_t csi_cb_count = 0;
static volatile uint32_t csi_drop_count = 0;

#if CSI_FORMAT_LOG_ENABLED
static void log_csi_format_if_changed(const wifi_pkt_rx_ctrl_t *rx_ctrl, int len)
{
    static bool have_last = false;
    static int last_len = -1;
    static unsigned last_sig_mode = 0;
    static unsigned last_cwb = 0;
    static unsigned last_channel = 0;
    static unsigned last_secondary_channel = 0;
    static uint32_t log_count = 0;

    unsigned sig_mode = rx_ctrl->sig_mode;
    unsigned cwb = rx_ctrl->cwb;
    unsigned channel = rx_ctrl->channel;
    unsigned secondary_channel = rx_ctrl->secondary_channel;
    bool changed = !have_last ||
        len != last_len ||
        sig_mode != last_sig_mode ||
        cwb != last_cwb ||
        channel != last_channel ||
        secondary_channel != last_secondary_channel;

    if (changed || log_count < 10) {
        ESP_LOGI(TAG,
                 "CSI format: len=%d pairs=%d sig_mode=%u cwb=%u channel=%u secondary=%u",
                 len, len / 2, sig_mode, cwb, channel, secondary_channel);
        log_count++;
    }

    have_last = true;
    last_len = len;
    last_sig_mode = sig_mode;
    last_cwb = cwb;
    last_channel = channel;
    last_secondary_channel = secondary_channel;
}
#endif

typedef struct {
    uint16_t seq;
    uint16_t n_amp;
    uint32_t ts_us;
    int8_t rssi;
    uint8_t agc_gain;
    int8_t fft_gain;
    float compensate_gain;
    int16_t amp[CSI_AMP_MAX_SUBCARRIERS];
} csi_amp_frame_t;

static void check_esp_err(const char *what, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", what, esp_err_to_name(err));
    }
}

static void udp_listen_task(void *pvParameter)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP listen socket failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(RX_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP bind failed: %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP listener started on port %d", RX_PORT);
    uint32_t recv_count = 0;
    int64_t last_report_us = esp_timer_get_time();

    while (1) {
        char buf[128];
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        if (len < 0) {
            ESP_LOGW(TAG, "UDP recv failed: %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        buf[len] = '\0';
        recv_count++;

        if (strcmp(buf, "DISCOVER_CSI_RX") == 0) {
            const char reply[] = "CSI_RX_HERE";
            sendto(sock, reply, strlen(reply), 0, (struct sockaddr *)&source_addr, socklen);
            ESP_LOGI(TAG, "Discovery reply sent to %s:%d",
                     inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
            continue;
        }

#if UDP_RATE_LOG_ENABLED
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= 1000000) {
            ESP_LOGI(TAG, "UDP RX rate: %lu pkt/s, last from %s:%d",
                     recv_count, inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
            recv_count = 0;
            last_report_us = now_us;
        }
#endif
    }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (!info || !info->buf || !csi_queue) {
        return;
    }

    if (ap_bssid_valid && memcmp(info->mac, ap_bssid, sizeof(ap_bssid)) != 0) {
        return;
    }

    const wifi_pkt_rx_ctrl_t* rx_ctrl = &info->rx_ctrl;
#if CSI_FORMAT_LOG_ENABLED
    log_csi_format_if_changed(rx_ctrl, info->len);
#endif
    static int s_count = 0;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;

    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < RX_GAIN_BASELINE_PACKETS) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == RX_GAIN_BASELINE_PACKETS) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
        ESP_LOGI(TAG, "RX gain baseline: agc=%u fft=%d", agc_gain_baseline, fft_gain_baseline);
    }

    if (s_count >= RX_GAIN_BASELINE_PACKETS) {
        esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    }
    s_count++;
    (void)s_count;

    static uint16_t seq = 0;
    csi_amp_frame_t frame = {
        .seq = seq++,
        .ts_us = (uint32_t)rx_ctrl->timestamp,
        .rssi = rx_ctrl->rssi,
        .agc_gain = agc_gain,
        .fft_gain = fft_gain,
        .compensate_gain = compensate_gain,
    };

    int pairs = info->len / 2;
    if (pairs > CSI_AMP_MAX_SUBCARRIERS) {
        pairs = CSI_AMP_MAX_SUBCARRIERS;
    }

    int start_pair = 0;
    if (pairs > CSI_PRINT_SUBCARRIERS) {
        start_pair = (pairs - CSI_PRINT_SUBCARRIERS) / 2;
        pairs = CSI_PRINT_SUBCARRIERS;
    }

    frame.n_amp = (uint16_t)pairs;
    for (int i = 0; i < pairs; i++) {
        int idx = 2 * (start_pair + i);
        int8_t csi_i = info->buf[idx];
        int8_t csi_q = info->buf[idx + 1];
        frame.amp[i] = (int16_t)lrintf(sqrtf((float)csi_i * csi_i + (float)csi_q * csi_q));
    }

    csi_cb_count++;
    if (xQueueSend(csi_queue, &frame, 0) != pdTRUE) {
        csi_drop_count++;
    }
}

static void csi_uart_task(void *pvParameter)
{
    csi_amp_frame_t frame;
    uint32_t sent_count = 0;
    uint32_t last_cb_count = 0;
    uint32_t last_drop_count = 0;
    int64_t last_report_us = esp_timer_get_time();

    while (1) {
        if (xQueueReceive(csi_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        printf("AMP:%u:%u:%d:%.6f:%d",
               frame.seq, frame.agc_gain, frame.fft_gain, frame.compensate_gain, frame.amp[0]);
        for (int i = 1; i < frame.n_amp; i++) {
            printf(",%d", frame.amp[i]);
        }
        printf("\n");
        fflush(stdout);
        sent_count++;

#if CSI_RATE_LOG_ENABLED
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= 1000000) {
            ESP_LOGI(TAG, "CSI UART rate: %lu pkt/s, cb=%lu, drop=%lu, rssi=%d",
                     sent_count, csi_cb_count - last_cb_count,
                     csi_drop_count - last_drop_count, frame.rssi);
            sent_count = 0;
            last_cb_count = csi_cb_count;
            last_drop_count = csi_drop_count;
            last_report_us = now_us;
        }
#endif
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        wifi_connected = false;
        ap_bssid_valid = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "RX IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            memcpy(ap_bssid, ap_info.bssid, sizeof(ap_bssid));
            ap_bssid_valid = true;
            ESP_LOGI(TAG, "Filtering CSI from AP BSSID: %02x:%02x:%02x:%02x:%02x:%02x",
                     ap_bssid[0], ap_bssid[1], ap_bssid[2],
                     ap_bssid[3], ap_bssid[4], ap_bssid[5]);
        }
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    csi_queue = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_amp_frame_t));
    if (!csi_queue) {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }

    xTaskCreatePinnedToCore(csi_uart_task, "csi_uart", 4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(udp_listen_task, "udp_listen", 4096, NULL, 5, NULL, 0);

    wifi_csi_config_t csi_config = {
        .lltf_en = 1,
        .htltf_en = 1,
        .stbc_htltf2_en = 1,
        .ltf_merge_en = 0,
        .channel_filter_en = 1,
        .manu_scale = 0,
        .shift = 0,
    };

    check_esp_err("esp_wifi_set_promiscuous(true)", esp_wifi_set_promiscuous(true));
    check_esp_err("esp_wifi_set_csi_config", esp_wifi_set_csi_config(&csi_config));
    check_esp_err("esp_wifi_set_csi_rx_cb", esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    check_esp_err("esp_wifi_set_csi(true)", esp_wifi_set_csi(true));
    ESP_LOGI(TAG, "CSI AMP RX started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
