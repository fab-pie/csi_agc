/*
    logs
*/
#define TAG "ZF_TX_UTILS"
#define FILE_ID 0x28
#include "zoecare/logs/logs.h"

#include "zoecare/tx/zoecare_utils.h"

#include "zoecare/tx/config.h"
#include "zoecare/tx/zoecare_functions.h"

#include <cJSON.h> // JSON parsing and generation library

float get_chip_temperature(temperature_sensor_handle_t temp_handle)
{
    // Enable temperature sensor
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    // Get converted sensor data
    float tsens_out;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
    // Disable the temperature sensor if it is not needed and save the power
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

    return tsens_out;
}

float calculate_cpu_load(void)
{
    char buffer[1024];            // Buffer to hold task runtime stats
    vTaskGetRunTimeStats(buffer); // Get runtime stats for all tasks

    float idle_percentage_core_0 = 0.0f;
    float idle_percentage_core_1 = 0.0f;
    float total_idle_percentage = 0.0f;

    // Parse the buffer line by line
    char *line = strtok(buffer, "\n");
    while (line != NULL) {
        char task_name[20];
        char percentage_str[10];

        // Read task name and percentage
        sscanf(line, "%19s %*d %9s", task_name, percentage_str);

        // Check if it's an idle task and extract the percentage
        if (strcmp(task_name, "IDLE0") == 0) {
            idle_percentage_core_0 =
                atof(percentage_str); // Convert percentage to float
        } else if (strcmp(task_name, "IDLE1") == 0) {
            idle_percentage_core_1 =
                atof(percentage_str); // Convert percentage to float
        }

        // Move to the next line
        line = strtok(NULL, "\n");
    }

    // Calculate total idle percentage
    total_idle_percentage =
        (idle_percentage_core_0 + idle_percentage_core_1) / 2.0f;

    // Calculate CPU load
    float cpu_load = 100.0f - total_idle_percentage;

    return cpu_load; // Return the CPU load
}

void turn_on_led(void) { gpio_set_level(LED_GPIO, 0); }
void turn_off_led(void) { gpio_set_level(LED_GPIO, 1); }

/**
 * @brief replace all spaces in a string with underscores
 *
 * @param str
 */
void replace_spaces_with_underscores(char *str)
{
    while (*str) {
        if (*str == ' ') {
            *str = '_';
        }
        str++;
    }
}

/**
 * @brief Verify if the pointer allocation is correct.
 *
 * @param size size of wanted allocation
 * @return void* pointer to memory allocated
 */
void *checked_malloc(size_t size)
{
    void *ptr = malloc(size);
    if (ptr == NULL) {
        LOGE("Failed to allocate %zu bytes", size);
        esp_restart();
    }
    return ptr;
}

/**
 * @brief Verify if the pointer allocation is correct.
 *
 * @param ptr already allocated pointer
 * @param size size of pointer allocation
 */
void check_malloc(void *ptr, size_t size)
{
    if (ptr == NULL) {
        LOGE("Failed to allocate %zu bytes", size);
        esp_restart();
    }
    return;
}

/**
 * @brief Verify if the free is done.
 *
 * @param ptr already allocated pointer
 */
void checked_free(void *ptr)
{
    if (ptr == NULL) {
        LOGE("ptr has already been freed");
        return;
    }
    free(ptr);
    ptr = NULL;
}

/**
 * @brief Remove all quotes from a string
 *
 * @param str
 */
void remove_quote(char *str)
{
    char *p = str;
    while (*p) {
        if (*p == '"' || *p == '\xe2') {
            *p = ' ';
        }
        p++;
    }
}

/**
 * @brief extract the unixtime from a JSON response
 *
 * @param json_response
 * @return int
 */
int extract_unixtime(const char *json_response)
{
    cJSON *json = cJSON_Parse(json_response);
    if (json == NULL) {
        LOGE("Erreur lors de l'analyse du JSON");
        return -1;
    }

    cJSON *unixtime = cJSON_GetObjectItemCaseSensitive(json, "unixtime");
    if (!cJSON_IsNumber(unixtime)) {
        LOGE("Le champ unixtime est introuvable ou non valide");
        cJSON_Delete(json);
        return -1;
    }

    int timestamp = unixtime->valueint;
    cJSON_Delete(json);
    return timestamp;
}

/**
 * @brief Set the system time object
 *
 * @param unix_time
 */
void set_system_time(int unix_time)
{
    struct timeval tv;
    tv.tv_sec = unix_time; // Seconds
    tv.tv_usec = 0;        // Microseconds

    // Set the system time
    if (settimeofday(&tv, NULL) != 0) {
        LOGE("Erreur lors de la définition de l'heure système");
    } else {
        LOGI("Heure système définie avec succès");
    }

    // Print the current time
    time_t now = time(NULL);
    LOGI("Heure actuelle : %s", ctime(&now));
}
