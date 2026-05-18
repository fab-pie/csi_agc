#ifndef ZOECARE_FUNCTIONS_H
#define ZOECARE_FUNCTIONS_H

#include <stdbool.h>

void configure_led(void);
void csi_counter_task(void *pvParameters);
void isAliveTask(void *pvParameters);
void send_transmission_task(void *pvParameters);
void send_transmission_task2(void *pvParameters);
int scan_best_channel(void);

#endif // ZOECARE_FUNCTIONS_H