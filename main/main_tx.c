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
#include "esp_netif_ip_addr.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define TX_AP_SSID "CSI_TX_AP"
#define TX_AP_PASS "csi123456"
#define TX_AP_CHANNEL 6
#define TX_AP_MAX_CONN 4

/* Optional upstream Wi-Fi for later non-serial telemetry.
 * Keep 0 for isolated line-of-sight CSI tests. */
#define TX_ENABLE_UPSTREAM_STA 0
#define UPSTREAM_WIFI_SSID "DVIPro"
#define UPSTREAM_WIFI_PASS "Jrd729kz"

#define TX_PORT 12345
#define DISCOVERY_ADDR "255.255.255.255"
#define DISCOVERY_RETRIES 20
#define DISCOVERY_TIMEOUT_MS 500

static const char *TAG = "csi_tx";

static bool wifi_connected = false;

static bool discover_rx(int sock, struct sockaddr_in *dest_addr)
{
    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct timeval timeout = {
        .tv_sec = 0,
        .tv_usec = DISCOVERY_TIMEOUT_MS * 1000,
    };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in discovery_addr = {
        .sin_addr.s_addr = inet_addr(DISCOVERY_ADDR),
        .sin_family = AF_INET,
        .sin_port = htons(TX_PORT),
    };

    const char probe[] = "DISCOVER_CSI_RX";
    char reply[32];

    for (int attempt = 1; attempt <= DISCOVERY_RETRIES; attempt++) {
        sendto(sock, probe, strlen(probe), 0,
               (struct sockaddr *)&discovery_addr, sizeof(discovery_addr));

        struct sockaddr_in source_addr;
        socklen_t source_len = sizeof(source_addr);
        int len = recvfrom(sock, reply, sizeof(reply) - 1, 0,
                           (struct sockaddr *)&source_addr, &source_len);
        if (len > 0) {
            reply[len] = '\0';
            if (strcmp(reply, "CSI_RX_HERE") == 0) {
                dest_addr->sin_addr = source_addr.sin_addr;
                dest_addr->sin_family = AF_INET;
                dest_addr->sin_port = htons(TX_PORT);
                ESP_LOGI(TAG, "Discovered RX at %s:%d",
                         inet_ntoa(dest_addr->sin_addr), TX_PORT);
                return true;
            }
        }

        ESP_LOGW(TAG, "RX discovery attempt %d/%d failed", attempt, DISCOVERY_RETRIES);
    }

    return false;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
#if TX_ENABLE_UPSTREAM_STA
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi STA started, connecting...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Upstream WiFi disconnected, reconnecting...");
        wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "TX upstream IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
    }
#else
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station connected: %02x:%02x:%02x:%02x:%02x:%02x aid=%d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5], event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGW(TAG, "Station disconnected: %02x:%02x:%02x:%02x:%02x:%02x aid=%d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5], event->aid);
    }
#endif
}

static void tx_task(void *pvParameter)
{
    uint8_t ap_mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_AP, ap_mac);
    ESP_LOGI(TAG, "TX AP MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             ap_mac[0], ap_mac[1], ap_mac[2], ap_mac[3], ap_mac[4], ap_mac[5]);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest_addr = {0};
    while (!discover_rx(sock, &dest_addr)) {
        ESP_LOGW(TAG, "RX not found, retrying discovery...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    int counter = 0;
    uint32_t sent_count = 0;
    int64_t last_report_us = esp_timer_get_time();
    char payload[64];
    ESP_LOGI(TAG, "Starting transmission to %s:%d...",
             inet_ntoa(dest_addr.sin_addr), TX_PORT);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        snprintf(payload, sizeof(payload), "TX_PKT_%d", counter++);
        int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGW(TAG, "sendto failed: errno %d", errno);
        } else {
            sent_count++;
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_report_us >= 1000000) {
                ESP_LOGI(TAG, "TX rate: %lu pkt/s, last=%s", sent_count, payload);
                sent_count = 0;
                last_report_us = now_us;
            }
        }

        /* Sleep until next 10ms slot (target 100Hz) */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

        /* periodic 10ms sleep */
    }

    if (sock != -1) {
        close(sock);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if TX_ENABLE_UPSTREAM_STA
    esp_netif_create_default_wifi_sta();
#endif
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_ip_info_t ap_ip;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(ap_netif, &ap_ip));
    ESP_LOGI(TAG, "TX AP IP: " IPSTR, IP2STR(&ap_ip.ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL, NULL));
#if TX_ENABLE_UPSTREAM_STA
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL, NULL));
#endif

    wifi_config_t ap_config = {
        .ap = {
            .ssid = TX_AP_SSID,
            .password = TX_AP_PASS,
            .ssid_len = strlen(TX_AP_SSID),
            .channel = TX_AP_CHANNEL,
            .max_connection = TX_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    if (strlen(TX_AP_PASS) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

#if TX_ENABLE_UPSTREAM_STA
    wifi_config_t sta_config = {
        .sta = {
            .ssid = UPSTREAM_WIFI_SSID,
            .password = UPSTREAM_WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
#else
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
#endif
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "CSI TX AP started: ssid=%s channel=%d", TX_AP_SSID, TX_AP_CHANNEL);

#if TX_ENABLE_UPSTREAM_STA
    ESP_LOGI(TAG, "TX upstream STA enabled: connecting to %s", UPSTREAM_WIFI_SSID);
    while (!wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
#endif

    xTaskCreate(tx_task, "tx_task", 4096, NULL, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
