/*
    logs
*/
#define TAG "ZF_TX_INIT"
#define FILE_ID 0x27
#include "zoecare/logs/logs.h"

#include "zoecare/tx/zoecare_init.h"

#include "zoecare/tx/config.h"
#include "zoecare/tx/zoecare_functions.h"
#include "zoecare/tx/zoecare_utils.h"


#include <esp_sntp.h>       // SNTP client for time synchronization
#include <esp_task_wdt.h>   // esp_task_wdt_reset()

/**
 * @brief Handler for the HTTP event to get the current time.
 *
 * @param evt event
 * @return esp_err_t error code
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
            LOGI("Réponse JSON : %s", buffer);
            int unixtime = extract_unixtime(buffer);
            if (unixtime != -1) {
                LOGI("Unixtime : %d", unixtime);
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
 * @brief Send an HTTP GET request to the worldtimeapi.org API to get the
 * current time.
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
        LOGI("HTTP GET Status = %d, content_length = %lld",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    } else {
        LOGE("Échec de la requête HTTP GET : %s", esp_err_to_name(err));
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
        LOGI("Waiting for system time to be set... (%d/%d)", retry, retry_count);
        // Feed WDT in 1 s chunks: a plain vTaskDelay(5000) without any reset
        // would exceed the TWDT window if called from isAliveTask.
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
