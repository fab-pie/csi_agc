/*
 * fall_validation.c - Post-fall validation engine for ESP32.
 *
 * Ports the Python multi_score_realtime.py validation logic:
 *   1. compound_fall_rule(): (any>0.75) OR (2 consec>0.5) OR (4 consec>0)
 *   2. Motion-absence detection via P25 percentile on baseline std(DIAG_LT)
 *   3. State machine: IDLE → WAITING → OBSERVING → VALIDATED/REJECTED
 *
 * All parameters match the field-tested Python defaults.
 */

#include "zoecare/rx/fall_validation.h"
#include <string.h>
#include <math.h>


// Internal helpers (forward declarations) 

static void fv_freeze_baseline(fv_engine_t *eng);
static void fv_complete_event(fv_engine_t *eng, fv_status_t final_status);


void fv_engine_init(fv_engine_t *eng)
{
    memset(eng, 0, sizeof(*eng));
    eng->state = FV_STATUS_IDLE;
    eng->cooldown_until = 0.0f;
}


// fv_compound_fall_rule
// Direct port of Python compound_fall_rule() (multi_score_realtime.py:281)  

fv_compound_result_t fv_compound_fall_rule(const float *scores, int n)
{
    fv_compound_result_t result = { .fell = false, .reason = "" };

    // Rule 1: any score > 0.75 
    for (int i = 0; i < n; i++) {
        if (scores[i] > 0.75f) {
            result.fell = true;
            result.reason = "score>0.75";
            return result;
        }
    }

    // Rule 2: 2 consecutive > 0.5 
    int c05 = 0;
    for (int i = 0; i < n; i++) {
        if (scores[i] > 0.5f) {
            if (++c05 >= 2) {
                result.fell = true;
                result.reason = "2consec>0.5";
                return result;
            }
        } else {
            c05 = 0;
        }
    }

    // Rule 3: 4 consecutive > 0.0 
    int c0 = 0;
    for (int i = 0; i < n; i++) {
        if (scores[i] > 0.0f) {
            if (++c0 >= 4) {
                result.fell = true;
                result.reason = "4consec>0";
                return result;
            }
        } else {
            c0 = 0;
        }
    }

    return result;
}


// Matches np.percentile(data, p) with default "linear" interpolation.       
// Insertion sort on stack copy - O(n^2) but n <= 60, so ~10 us max.         

float fv_percentile(const float *data, int n, int percentile)
{
    if (n <= 0) return 0.0f;
    if (n == 1) return data[0];

    // Copy to stack buffer for sorting 
    float tmp[FV_BASELINE_WINDOW];
    int len = (n > FV_BASELINE_WINDOW) ? FV_BASELINE_WINDOW : n;
    memcpy(tmp, data, len * sizeof(float));

    // Insertion sort 
    for (int i = 1; i < len; i++) {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    // Linear interpolation (matches numpy default) 
    float rank = (float)percentile / 100.0f * (float)(len - 1);
    int lo = (int)rank;
    int hi = lo + 1;
    if (hi >= len) hi = len - 1;
    float frac = rank - (float)lo;
    return tmp[lo] + frac * (tmp[hi] - tmp[lo]);
}


void fv_baseline_push(fv_engine_t *eng, float motion_std)
{
    fv_motion_ring_t *r = &eng->baseline;
    r->buf[r->head] = motion_std;
    r->head = (r->head + 1) % FV_BASELINE_WINDOW;
    if (r->count < FV_BASELINE_WINDOW)
        r->count++;
}


// fv_engine_update
// State machine: called once per DAPD cycle (~1 Hz).                        

void fv_engine_update(fv_engine_t *eng, float now_s,
                      bool fell, const char *reason,
                      float max_score, float motion_std)
{
    switch (eng->state) {

    case FV_STATUS_IDLE:
        // Accumulate consecutive compound-rule detections 
        if (fell) {
            eng->consec_count++;
        } else {
            eng->consec_count = 0;
        }

        // Trigger fall event when threshold reached and cooldown expired 
        if (eng->consec_count >= FV_N_CONSEC_REQUIRED
                && now_s >= eng->cooldown_until) {
            eng->active_event.detection_time = now_s;
            eng->active_event.max_score      = max_score;
            eng->active_event.status         = FV_STATUS_WAITING;
            eng->active_event.obs_start_time = now_s + FV_T_WAIT;
            eng->active_event.obs_end_time   = now_s + FV_T_WAIT + FV_T_OBS;
            eng->active_event.still_count    = 0;
            eng->active_event.total_obs      = 0;
            eng->active_event.trigger_reason = reason;

            // Freeze baseline snapshot and compute P25 threshold 
            fv_freeze_baseline(eng);

            eng->state           = FV_STATUS_WAITING;
            eng->cooldown_until  = eng->active_event.obs_end_time;
            eng->consec_count    = 0;
            eng->obs_still_count = 0;
            eng->obs_total_count = 0;
        }
        break;

    case FV_STATUS_WAITING:
        if (now_s >= eng->active_event.obs_start_time) {
            eng->state = FV_STATUS_OBSERVING;
            // Fall through to immediately process this sample 
        } else {
            break;
        }
        // intentional fallthrough 
        __attribute__((fallthrough));

    case FV_STATUS_OBSERVING:
        // Check for insufficient baseline 
        if (eng->frozen_baseline_len < FV_MIN_BASELINE) {
            if (now_s >= eng->active_event.obs_end_time) {
                fv_complete_event(eng, FV_STATUS_NO_DATA);
            }
            break;
        }

        // Count this observation 
        eng->obs_total_count++;
        if (motion_std < eng->frozen_threshold) {
            eng->obs_still_count++;
        }

        // Update active event counters (for telemetry access) 
        eng->active_event.still_count = eng->obs_still_count;
        eng->active_event.total_obs   = eng->obs_total_count;

        // Check if observation window complete 
        if (now_s >= eng->active_event.obs_end_time) {
            if (eng->obs_total_count == 0) {
                fv_complete_event(eng, FV_STATUS_NO_DATA);
            } else {
                float ratio = (float)eng->obs_still_count
                            / (float)eng->obs_total_count;
                if (ratio >= FV_F_STILL) {
                    fv_complete_event(eng, FV_STATUS_VALIDATED);
                } else {
                    fv_complete_event(eng, FV_STATUS_REJECTED);
                }
            }
        }
        break;

    case FV_STATUS_VALIDATED:
    case FV_STATUS_REJECTED:
    case FV_STATUS_NO_DATA:
        // Terminal states: return to IDLE on next cycle 
        eng->state = FV_STATUS_IDLE;
        break;
    }
}


// fv_status_str

const char *fv_status_str(fv_status_t status)
{
    switch (status) {
    case FV_STATUS_IDLE:      return "IDLE";
    case FV_STATUS_WAITING:   return "WAITING";
    case FV_STATUS_OBSERVING: return "OBSERVING";
    case FV_STATUS_VALIDATED: return "VALIDATED";
    case FV_STATUS_REJECTED:  return "REJECTED";
    case FV_STATUS_NO_DATA:   return "NO_DATA";
    default:                  return "UNKNOWN";
    }
}


// Internal helpers

static void fv_freeze_baseline(fv_engine_t *eng)
{
    fv_motion_ring_t *r = &eng->baseline;
    eng->frozen_baseline_len = r->count;

    // Extract ring buffer in chronological order 
    int start = (r->count < FV_BASELINE_WINDOW) ? 0 : r->head;
    for (int i = 0; i < r->count; i++) {
        eng->frozen_baseline[i] = r->buf[(start + i) % FV_BASELINE_WINDOW];
    }

    // Compute P25 threshold from frozen baseline 
    if (r->count >= FV_MIN_BASELINE) {
        eng->frozen_threshold = fv_percentile(
            eng->frozen_baseline, eng->frozen_baseline_len, FV_PERCENTILE);
    } else {
        eng->frozen_threshold = 0.0f;
    }
}

static void fv_complete_event(fv_engine_t *eng, fv_status_t final_status)
{
    eng->active_event.status = final_status;
    eng->state = final_status;

    // Store in event history ring buffer 
    eng->events[eng->event_head] = eng->active_event;
    eng->event_head = (eng->event_head + 1) % FV_MAX_EVENTS;
    if (eng->event_count < FV_MAX_EVENTS)
        eng->event_count++;
}
