#ifndef TX_CRED_SWITCH_H
#define TX_CRED_SWITCH_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Initialise the staged upstream credential switch module.
//
// Call once from app_main AFTER wifi_init() and after zc_zf_tx_cfg_load().
// If NVS contains staged credentials (up_ssid_new / up_pass_new), a background
// task is spawned that tests the new credentials and either commits them (sending
// TB telemetry {"cred_switch":"success"}) or reverts to the original ones
// (sending {"cred_switch":"failed"}).
//
// wifi_eg: the FreeRTOS EventGroupHandle used for WIFI_CONNECTED_BIT.  Pass the
// same group used by wifi_event_handler so the switch task can detect IP acquisition.
void tx_cred_switch_init(EventGroupHandle_t wifi_eg);

#endif // TX_CRED_SWITCH_H
