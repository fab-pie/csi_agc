#ifndef UTILS_H
#define UTILS_H

#include "driver/temperature_sensor.h" // Driver for temperature sensor

int extract_unixtime(const char *json_response);
void set_system_time(int unix_time);
void remove_quote(char *str);
void replace_spaces_with_underscores(char *str);
float calculate_cpu_load(void);
float get_chip_temperature(temperature_sensor_handle_t temp_handle);
void checked_free(void *ptr);
void *checked_malloc(size_t size);
float mean_array(int *array, int size);

/* CHECKED_FREE: calls checked_free AND nulls the caller's pointer.
 * checked_free(ptr) alone only nulls its local copy - this macro
 * prevents use-after-free by zeroing the original variable. */
#define CHECKED_FREE(ptr) do { checked_free(ptr); (ptr) = NULL; } while (0)

#endif