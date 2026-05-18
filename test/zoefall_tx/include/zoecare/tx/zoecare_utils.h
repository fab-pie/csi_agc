#ifndef ZOECARE_UTILS_H
#define ZOECARE_UTILS_H

#include <driver/temperature_sensor.h> // Driver for the built-in temperature sensor

float get_chip_temperature(temperature_sensor_handle_t temp_handle);
float calculate_cpu_load(void);
void turn_on_led(void);
void turn_off_led(void);
void replace_spaces_with_underscores(char *str);
void *checked_malloc(size_t size);
void check_malloc(void *ptr, size_t size);
void checked_free(void *ptr);
void remove_quote(char *str);
int extract_unixtime(const char *json_response);
void set_system_time(int unix_time);

#endif // ZOECARE_UTILS_H