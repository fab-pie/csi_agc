/*
    logs
*/
#define TAG "ZF_TX_FUNC"
#define FILE_ID 0x26
#include "zoecare/logs/logs.h"
#include "zoecare/err/err.h"

#include "zoecare/tx/zoecare_functions.h"

#include "zoecare/tx/config.h"
#include "zoecare/tx/zoecare_init.h"
#include "zoecare/tx/zoecare_utils.h"
#include "zoecare/common/config.h"
#include "zoecare/wifi/wifi.h"
#include "zoecare/report/report.h"
#include "zoecare/tx/version_info.h"
#include "esp_app_desc.h"
#include "esp_system.h"
#include "zoecare/tx/ota_update.h"
#include "esp_task_wdt.h"

#define THRESHOLD 50 // CSI rate threshold for LED to turn on

// Chunk size for WDT-safe long sleeps. Must be < WDT_TIMEOUT_MS (30s dev / 5min prod).
#define ISALIVE_WDT_CHUNK_MS 10000

unsigned int is_alive_counter = 0;       // Counter for isAlive messages
size_t csi_last_isAlive = 0; // Last csi counter when isAlive was executed
size_t csi_count = 0;        // csi counter from start
bool got_timestamp = false;  // Flag to check if the timestamp has been obtained
int low_csi = 0;             // Counter for low CSI rate
int packet_sent = 0;         // Number of packets sent

/**
 * @brief Set the GPIO to turn on the LED
 *
 */
void configure_led(void)
{
    // gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, (0x00000002));

    LOGI("Example configured to turn on GPIO LED!");
    /* Set the GPIO as a push/pull output */
    turn_off_led();
}

/**
 * @brief Each second, check if the CSI rate is above the threshold and print
 * the CSI rate
 *
 * @param pvParameters
 */
void csi_counter_task(void *pvParameters)
{
    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);
    esp_task_wdt_add(xTaskGetCurrentTaskHandle()); // faire surveiller la tâche actuelle par WDT
    configure_led();
    int low_counter = 0;
    while (true) {
        // Turn off the LED if CSI rate is below threshold
        if (packet_sent < THRESHOLD) {
            turn_off_led();
            // Don't count low CSI during OTA: transmission is intentionally
            // paused by send_transmission_task while ota_tx_in_progress is set.
            if (!ota_tx_in_progress) {
                low_counter++;
            }
        } else {
            low_counter = 0;
            turn_on_led();
        }

        // * The low rate part is normally handled by is_alive but the timeout is 5 min (too long) and we turn off the isAlive for TX presence
        if (low_counter >= 15) { // low threshold provokes restarts at startup because init and connection to AP take a bit of time
            LOGE("CSI is too low, restart !");
            esp_restart();
        }

        printf("CSI/s: %d\n", packet_sent);
        packet_sent = 0;

        vTaskDelay(pdMS_TO_TICKS(1000));

        esp_task_wdt_reset(); // Feed the WDT
    }
}

/**
 * @brief Task to send an isAlive message to the server every report_status_delay
 *
 * @param pvParameters
 */
void isAliveTask(void *pvParameters)
{
    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);
    // WDT re-enabled with chunked sleep so long delays don't trigger it.
    // Root cause of original disable: empty TB token caused zc_report_send to
    // retry/block past the WDT window. Fixed by guarding all sends with a token check.
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    temperature_sensor_handle_t temp_handle = NULL;
    temperature_sensor_config_t temp_sensor = {
        .range_min = 20,
        .range_max = 100,
    };
    ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor, &temp_handle));

    // One-time startup telemetry: sent as soon as WiFi is up (zc_report_send
    // blocks internally until got_ip is true, so no explicit wait needed here).
    char startup_msg[96];
    snprintf(startup_msg, sizeof(startup_msg),
             "{\"firmware_version\":\"%s\",\"role\":\"TX\"}",
             esp_app_get_description()->version);
    LOGI("Startup telemetry: %s", startup_msg);
    if (strlen(zc_zf_tx_cfg.tb_access_token) > 0) {
        ZC_TRY(zc_report_send, startup_msg, strlen(startup_msg));
        ota_tx_mark_valid();

        // One-shot abnormal reset telemetry: logged every boot after a crash/WDT/brownout.
        esp_reset_reason_t rr = esp_reset_reason();
        if (rr == ESP_RST_TASK_WDT || rr == ESP_RST_INT_WDT ||
            rr == ESP_RST_PANIC    || rr == ESP_RST_BROWNOUT) {
            const char *rr_name = (rr == ESP_RST_TASK_WDT) ? "task_wdt" :
                                  (rr == ESP_RST_INT_WDT)  ? "int_wdt"  :
                                  (rr == ESP_RST_PANIC)     ? "panic"    : "brownout";
            char rr_msg[128];
            snprintf(rr_msg, sizeof(rr_msg),
                     "{\"event\":\"abnormal_reset\",\"reason\":\"%s\",\"role\":\"TX\"}", rr_name);
            LOGW("Abnormal reset detected: %s", rr_name);
            ZC_TRY(zc_report_send, rr_msg, strlen(rr_msg));
        }
    } else {
        LOGW("TB token empty, skipping startup telemetry");
    }
    esp_task_wdt_reset();

    while (true) {
        // Chunked sleep: feed WDT in ISALIVE_WDT_CHUNK_MS intervals so long
        // report periods (report_status_delay > WDT timeout) don't trigger WDT.
        int total_ms = (int)(zc_zf_tx_cfg.report_status_delay * 1000);
        while (total_ms > 0) {
            int chunk = (total_ms < ISALIVE_WDT_CHUNK_MS) ? total_ms : ISALIVE_WDT_CHUNK_MS;
            vTaskDelay(pdMS_TO_TICKS(chunk));
            esp_task_wdt_reset();
            total_ms -= chunk;
        }

        // Increment the counter and log a message
        is_alive_counter++;
        LOGI("isAlive task executed %d times", is_alive_counter);

        // Calculate the CSI since the last isAlive
        csi_last_isAlive = csi_count - csi_last_isAlive;

        float temp = get_chip_temperature(temp_handle);
        uint32_t free_ram = esp_get_free_heap_size();
        float cpu_load = calculate_cpu_load();

        wifi_ap_record_t ap_info;
        int8_t rssi = -1;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            rssi = ap_info.rssi;
        }

        // Build the JSON payload
        char message[256];
        snprintf(message, sizeof(message),
                 "{\"temperature\":%.2f,\"ram_left\":%.2f,\"cpu_load\":%.2f,"
                 "\"csi_since_last_is_alive\":%d,\"rssi\":%d}",
                 temp, (float)free_ram / 1000, cpu_load, csi_last_isAlive,
                 rssi);

        if (csi_last_isAlive <= 3000)
            low_csi++;
        else
            low_csi = 0;
        if (low_csi >= 5) {
            LOGE("CSI is too low, RESTART");
            esp_restart();
        }

        csi_last_isAlive = csi_count;

        if (rssi == -1) {
            LOGE("Failed to get AP info, message: %s", message);
            continue;
        }

        if (strlen(zc_zf_tx_cfg.tb_access_token) == 0) {
            LOGW("TB token empty, skipping periodic telemetry");
            continue;
        }

        LOGI("isAlive payload: %s", message);
        ZC_TRY(zc_report_send, message, strlen(message));

        if (!got_timestamp) {
            obtain_time();
        }

        LOGI("isAlive task finished");
    }
}

char *data = (char *)"1\n";
/**
 * @brief Send a transmission to the IP set manually at high frequency. UDP
 * protocol
 *
 */
void send_transmission_task(void *pvParameters)
{
    ZC_ERR_SETUP_BACKTRACE_IN_TLS(1);
    esp_task_wdt_add(xTaskGetCurrentTaskHandle()); // faire surveiller la tâche actuelle par WDT
    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        int socket_fd = -1;
        char *ip = (char *)"192.168.4.2";
        struct sockaddr_in caddr;
        caddr.sin_family = AF_INET;
        caddr.sin_port = htons(2223);
        printf("initial wifi connection established.\n");
        if (inet_aton(ip, &caddr.sin_addr) == 0) {
            printf("ERROR: inet_aton\n");
            continue;
        }

        socket_fd = socket(PF_INET, SOCK_DGRAM, 0);
        if (socket_fd == -1) {
            printf("ERROR: Socket creation error [%s]\n", strerror(errno));
            continue;
        }
        struct timeval timeout;
        timeout.tv_sec = 5; // set a 5-second timeout for send operations
        timeout.tv_usec = 0;
        if (setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                       sizeof(timeout)) < 0) {
            printf("ERROR: Failed to set socket send timeout [%s]\n",
                   strerror(errno));
            close(socket_fd);
            continue;
        }
        if (connect(socket_fd, (const struct sockaddr *)&caddr,
                    sizeof(struct sockaddr)) == -1) {
            printf("ERROR: socket connection error [%s]\n", strerror(errno));
            // close socket
            close(socket_fd);
            continue;
        }

        while (1) {
            // Pause transmission during OTA download to free bandwidth
            if (ota_tx_in_progress) {
                esp_task_wdt_reset();
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            ssize_t result =
                sendto(socket_fd, data, strlen(data), 0,
                       (const struct sockaddr *)&caddr, sizeof(caddr));
            if (result ==
                -1) { // Slow down the transmission rate and not necessary
            } else if (result != strlen(data)) {
                printf("WARNING: Incomplete packet sent.\n");
            } else {
                packet_sent++;
                csi_count++;
                vTaskDelay(10 / portTICK_PERIOD_MS);

                esp_task_wdt_reset(); // Feed the WDT
            }
        }
        close(socket_fd);

        esp_task_wdt_reset(); // Feed the WDT
    }
}
