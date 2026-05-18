#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <stdbool.h>

// Flag shared with dapd_calc_task - when true, DAPD pauses to free CPU for OTA
extern volatile bool ota_in_progress;

// Start the periodic OTA check task and safety watchdog.
// Call from app_main() after WiFi is connected.
void ota_start_task(void);

// Mark the running firmware as valid, cancelling any pending rollback.
// Call after the first successful SVM inference.
void ota_mark_valid(void);

// Return the running firmware version string from esp_app_desc_t.
const char *ota_get_running_version(void);

// Verify that critical partitions exist at the offsets hardcoded at compile time.
// Call early in app_main, after nvs_flash_init. On mismatch + PENDING_VERIFY,
// triggers immediate rollback. On mismatch on a validated build, logs an error.
void ota_check_partition_layout(void);

#endif // OTA_UPDATE_H
