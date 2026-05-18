#ifndef ZF_TX_OTA_UPDATE_H
#define ZF_TX_OTA_UPDATE_H

#include <stdbool.h>

// Set to true while a firmware download is in progress.
// send_transmission_task checks this flag and pauses UDP transmission to free
// bandwidth for the OTA download.
extern volatile bool ota_tx_in_progress;

/*
 * Start the periodic OTA check task and safety watchdog.
 * Call from app_main() after WiFi (STA) is connected to the upstream network.
 *
 * Validation: after boot, the watchdog waits OTA_TX_VALIDATION_TIMEOUT_MS.
 * If the CSI counter has not incremented within that window, the firmware
 * is marked invalid and the device rolls back to the previous partition.
 */
void ota_tx_start_task(void);

/*
 * Mark the running firmware as valid, cancelling the pending rollback.
 * Call from the CSI task after the first batch of CSI packets is confirmed.
 */
void ota_tx_mark_valid(void);

/*
 * Return the running firmware version string from esp_app_desc_t.
 */
const char *ota_tx_get_running_version(void);

/*
 * Verify that critical partitions exist at the offsets hardcoded at compile time.
 * Call early in app_main, after initialize_nvs(). On mismatch + PENDING_VERIFY,
 * triggers immediate rollback. On mismatch on a validated build, logs an error.
 */
void ota_tx_check_partition_layout(void);

#endif // ZF_TX_OTA_UPDATE_H
