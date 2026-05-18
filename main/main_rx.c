#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

#define WIFI_SSID "DVIPro1"
#define WIFI_PASS "Jrd729kz"
#define CSI_BINARY_MAX_LEN 512
#define CSI_BINARY_SEND_MAX_LEN 256
#define CSI_QUEUE_LEN 32
#define UDP_RATE_LOG_ENABLED 0
#define CSI_STATS_LOG_ENABLED 0

static const char *TAG = "csi_rx";

static bool wifi_connected = false;
static QueueHandle_t csi_queue = NULL;
static volatile uint32_t csi_cb_count = 0;
static volatile uint32_t csi_drop_count = 0;

typedef struct {
    uint16_t len;
    int8_t rssi;
    uint8_t agc_gain;
    int8_t fft_gain;
    int16_t compensate_gain;
    uint64_t time_us;
    uint8_t mac[6];
    uint8_t data[CSI_BINARY_SEND_MAX_LEN];
} csi_frame_t;

static void check_esp_err(const char *what, esp_err_t err)
{
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed: %s", what, esp_err_to_name(err));
    }
}

/* UDP listener to verify packet reception independent of CSI */
static void udp_listen_task(void *pvParameter)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "UDP listen: socket failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "UDP listen: bind failed: %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP listener started on port %d", 12345);
    uint32_t recv_count = 0;
    int64_t last_report_us = esp_timer_get_time();

    while (1) {
        char buf[256];
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

#if UDP_RATE_LOG_ENABLED
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= 1000000) {
            ESP_LOGI(TAG, "UDP rate: %lu pkt/s, last from %s:%d",
                     recv_count, inet_ntoa(source_addr.sin_addr), ntohs(source_addr.sin_port));
            recv_count = 0;
            last_report_us = now_us;
        }
#else
        (void)recv_count;
        (void)last_report_us;
        (void)source_addr;
#endif
    }
}

static void wifi_csi_binary_rx_cb(void *ctx, wifi_csi_info_t *csi_info)
{
    if (!csi_info || !csi_queue) return;

    csi_cb_count++;

    csi_frame_t frame = {
        .len = csi_info->len > CSI_BINARY_SEND_MAX_LEN ? CSI_BINARY_SEND_MAX_LEN : csi_info->len,
        .rssi = csi_info->rx_ctrl.rssi,
        .time_us = (uint64_t)esp_timer_get_time(),
    };

    memcpy(frame.mac, csi_info->mac, sizeof(frame.mac));
    memcpy(frame.data, csi_info->buf, frame.len);

    if (xQueueSend(csi_queue, &frame, 0) != pdTRUE) {
        csi_drop_count++;
    }
}

static void csi_serial_task(void *pvParameter)
{
    csi_frame_t csi;
    uint8_t out[26 + CSI_BINARY_SEND_MAX_LEN];
    uint32_t sent_count = 0;
    uint32_t last_cb_count = 0;
    uint32_t last_drop_count = 0;
    int64_t last_report_us = esp_timer_get_time();

    while (1) {
        if (xQueueReceive(csi_queue, &csi, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        out[0] = 'C';
        out[1] = 'S';
        out[2] = 'I';
        out[3] = 'B';
        out[4] = 2;
        out[5] = (uint8_t)(csi.len & 0xff);
        out[6] = (uint8_t)((csi.len >> 8) & 0xff);
        for (int i = 0; i < 8; i++) {
            out[7 + i] = (uint8_t)((csi.time_us >> (8 * i)) & 0xff);
        }
        out[15] = (uint8_t)csi.rssi;
        memcpy(&out[16], csi.mac, sizeof(csi.mac));
        out[22] = csi.agc_gain;
        out[23] = (uint8_t)csi.fft_gain;
        out[24] = (uint8_t)(csi.compensate_gain & 0xff);
        out[25] = (uint8_t)((csi.compensate_gain >> 8) & 0xff);
        memcpy(&out[26], csi.data, csi.len);

        fwrite(out, 1, 26 + csi.len, stdout);
        fflush(stdout);
        sent_count++;

#if CSI_STATS_LOG_ENABLED
        int64_t now_us = esp_timer_get_time();
        if (now_us - last_report_us >= 5000000) {
            uint32_t cb_now = csi_cb_count;
            uint32_t drop_now = csi_drop_count;
            ESP_LOGI(TAG, "CSI stats 5s: cb=%lu sent=%lu drop=%lu",
                     cb_now - last_cb_count, sent_count, drop_now - last_drop_count);
            last_cb_count = cb_now;
            last_drop_count = drop_now;
            sent_count = 0;
            last_report_us = now_us;
        }
#else
        (void)sent_count;
        (void)last_cb_count;
        (void)last_drop_count;
        (void)last_report_us;
#endif
    }
}

#if 0
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *csi_info)
{
    if (!csi_info) return;

    uint8_t agc_gain = 0;
    int8_t fft_gain = 0;
    int16_t compensate_gain = 0;

    esp_err_t ret = ESP_ERR_NOT_SUPPORTED;
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to get rx gain: %d", ret);
    }

    (void)agc_gain;
    (void)fft_gain;

    ret = ESP_ERR_NOT_SUPPORTED;
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "failed to get compensation: %d", ret);
    }

    int len = csi_info->len;

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

#endif

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
        wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
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
    esp_event_handler_instance_t instance_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_ip));

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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    /* wait until connected and got IP */
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    csi_queue = xQueueCreate(CSI_QUEUE_LEN, sizeof(csi_frame_t));
    if (!csi_queue) {
        ESP_LOGE(TAG, "Failed to create CSI queue");
        return;
    }
    xTaskCreatePinnedToCore(csi_serial_task, "csi_serial", 4096, NULL, 6, NULL, 1);

    // Configure CSI after association
    wifi_csi_config_t csi_config = {
        .lltf_en = 1,
        .htltf_en = 1,
        .stbc_htltf2_en = 1,
        .ltf_merge_en = 1,
        .channel_filter_en = 1,
        .manu_scale = 0,
        .shift = 0,
    };
    check_esp_err("esp_wifi_set_promiscuous(true)", esp_wifi_set_promiscuous(true));
    check_esp_err("esp_wifi_set_csi_config", esp_wifi_set_csi_config(&csi_config));
    check_esp_err("esp_wifi_set_csi_rx_cb", esp_wifi_set_csi_rx_cb(wifi_csi_binary_rx_cb, NULL));
    check_esp_err("esp_wifi_set_csi(true)", esp_wifi_set_csi(true));

    ESP_LOGI(TAG, "CSI RX started on SSID: %s", WIFI_SSID);

    xTaskCreatePinnedToCore(udp_listen_task, "udp_listen", 4096, NULL, 5, NULL, 0);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
