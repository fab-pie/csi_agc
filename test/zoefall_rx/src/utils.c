/*
    logs
*/
#define TAG "ZF_RX_UTILS"
#define FILE_ID 0x1D
#include "zoecare/logs/logs.h"

#include "zoecare/rx/utils.h"
#include <stdbool.h>
#include <stddef.h>
#include "cJSON.h"   // JSON parsing and creation library
#include "esp_mac.h" // Functions to manage MAC addresses
#include <string.h>
#include "zoecare/rx/config.h"


/**
 * @brief Get the chip temperature object
 *
 * @param temp_handle
 * @return float
 */
float get_chip_temperature(temperature_sensor_handle_t temp_handle)
{
    // Enable temperature sensor
    ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
    // Get converted sensor data
    float tsens_out;
    ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
    //  Disable the temperature sensor if it is not needed and save the power
    ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

    return tsens_out;
}


/**
 * @brief Calculate the CPU load based on the task runtime statistics
 *
 * @return float CPU load in percentage
 */
float calculate_cpu_load(void)
{
    char *buffer =
        (char *)checked_malloc(1024); // Buffer to hold task runtime stats
    vTaskGetRunTimeStats(buffer);     // Get runtime stats for all tasks

    float idle_percentage_core_0 = 0.0f;
    float idle_percentage_core_1 = 0.0f;
    float total_idle_percentage = 0.0f;

    // Parse the buffer line by line - use stack buffers to avoid heap fragmentation
    char *line = strtok(buffer, "\n");
    while (line != NULL) {
        char task_name[20];
        char percentage_str[10];

        // Read task name and percentage
        if (sscanf(line, "%19s %*d %9s", task_name, percentage_str) == 2) {
            // Check if it's an idle task and extract the percentage
            if (strcmp(task_name, "IDLE0") == 0) {
                idle_percentage_core_0 = atof(percentage_str);
            } else if (strcmp(task_name, "IDLE1") == 0) {
                idle_percentage_core_1 = atof(percentage_str);
            }
        }

        // Move to the next line
        line = strtok(NULL, "\n");
    }

    // Calculate total idle percentage
    total_idle_percentage =
        (idle_percentage_core_0 + idle_percentage_core_1) / 2.0f;

    // Calculate CPU load
    float cpu_load = 100.0f - total_idle_percentage;
    checked_free(buffer); // Free the buffer
    return cpu_load;      // Return the CPU load
}


/**
 * @brief extract the unixtime from the JSON response
 *
 * @param json_response
 * @return int
 */
int extract_unixtime(const char *json_response)
{
    cJSON *json = cJSON_Parse(json_response);
    if (json == NULL) {
        LOGE("Error parsing JSON");
        return -1;
    }

    cJSON *unixtime = cJSON_GetObjectItemCaseSensitive(json, "unixtime");
    if (!cJSON_IsNumber(unixtime)) {
        LOGE(
                 "unixtime is not a number in the JSON response");
        cJSON_Delete(json);
        return -1;
    }

    int timestamp = unixtime->valueint;
    cJSON_Delete(json);
    return timestamp;
}


/**
 * @brief Set the system time to the given unix time
 *
 * @param unix_time
 */
void set_system_time(int unix_time)
{
    struct timeval tv;
    tv.tv_sec = unix_time; // Seconds
    tv.tv_usec = 0;        // Microseconds (not needed)

    if (settimeofday(&tv, NULL) != 0) {
        LOGE("Error setting system time");
    } else {
        LOGI("System time set successfully");
    }

    // Log the current time
    time_t now = time(NULL);
    LOGI("Time now : %s", ctime(&now));
}


/**
 * @brief remove quotes from a string
 *
 * @param str
 */
void remove_quote(char *str)
{
    char *p = str;
    while (*p) {
        if (*p == '"' || *p == '"') {
            *p = ' ';
        }
        p++;
    }
}


/**
 * @brief replace spaces with underscores in a string
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
 * @brief Verify if the free is done and increment free counter.
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
    // Note: ptr=NULL here would only null the local copy.
    // Use the CHECKED_FREE() macro (utils.h) to null the caller's pointer.
}


/**
 * @brief Verify if the pointer allocation is correct and increment malloc
 * counter.
 *
 * @param size size of wanted allocation
 * @return pointer to memory allocated
 */
void *checked_malloc(size_t size)
{
    void *ptr = malloc(size);
    if (ptr == NULL) {
        // handle_error(1, "MALLOC", "Failed to allocate %zu bytes", size); // TODO : same as checked_free
        LOGE("Failed to allocate %zu bytes", size);
    }
    return ptr;
}


float mean_array(int *array, int size) {
    float somme = 0;
    for (int i = 0; i < size; i ++){
        somme += array[i];
    }
    return somme / size;
}


