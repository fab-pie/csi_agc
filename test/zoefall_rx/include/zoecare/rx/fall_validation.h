#ifndef FALL_VALIDATION_H
#define FALL_VALIDATION_H

#include <stdint.h>
#include <stdbool.h>

// Compile-time parameters from multi_score_realtime.py

#ifndef FV_T_WAIT
#define FV_T_WAIT               2.0f        // seconds to wait after fall before observing
#endif
#ifndef FV_T_OBS
#define FV_T_OBS                45.0f       // observation window in seconds 
#endif
#ifndef FV_F_STILL
#define FV_F_STILL              0.60f       // required fraction of "still" observations 
#endif
#ifndef FV_N_CONSEC_REQUIRED
#define FV_N_CONSEC_REQUIRED    1           // consecutive compound-rule positives to trigger 
#endif
#ifndef FV_BASELINE_WINDOW
#define FV_BASELINE_WINDOW      60          // circular buffer size for motion baseline 
#endif
#ifndef FV_MIN_BASELINE
#define FV_MIN_BASELINE         15          // minimum baseline samples before validation 
#endif
#ifndef FV_PERCENTILE
#define FV_PERCENTILE           25          // P25 for motion-absence threshold 
#endif
#ifndef FV_MAX_EVENTS
#define FV_MAX_EVENTS           8           // max stored completed events 
#endif


// Validation status
typedef enum {
    FV_STATUS_IDLE      = 0,    // no fall detected, baseline accumulating 
    FV_STATUS_WAITING   = 1,    // fall detected, in t_wait cooldown 
    FV_STATUS_OBSERVING = 2,    // observation window active 
    FV_STATUS_VALIDATED = 3,    // fall confirmed (enough stillness) 
    FV_STATUS_REJECTED  = 4,    // fall rejected (motion continued) 
    FV_STATUS_NO_DATA   = 5,    // insufficient baseline data to decide 
} fv_status_t;


// Compound fall rule result (stateless, per-cycle)
typedef struct {
    bool        fell;
    const char *reason;         // points to string literal, no alloc needed 
} fv_compound_result_t;


// Fall event record
typedef struct {
    float       detection_time;     // seconds since boot (esp_timer) 
    float       max_score;          // highest SVM score at detection 
    fv_status_t status;
    float       obs_start_time;     // detection_time + t_wait 
    float       obs_end_time;       // obs_start_time + t_obs 
    int         still_count;
    int         total_obs;
    const char *trigger_reason;     // string literal from compound rule 
} fv_event_t;


// Motion baseline circular buffer
typedef struct {
    float   buf[FV_BASELINE_WINDOW];
    int     head;               // next write position 
    int     count;              // 0..FV_BASELINE_WINDOW 
} fv_motion_ring_t;


// Validation engine state
typedef struct {
    // State machine 
    fv_status_t         state;
    fv_event_t          active_event;       // valid when state != IDLE 
    int                 consec_count;       // consecutive compound-rule positives 
    float               cooldown_until;     // time until which no new event can trigger 

    // Motion baseline (pre-fall values, continuously updated) 
    fv_motion_ring_t    baseline;

    // Observation counters (accumulated during OBSERVING) 
    int                 obs_still_count;
    int                 obs_total_count;

    // Completed event history (ring buffer) 
    fv_event_t          events[FV_MAX_EVENTS];
    int                 event_head;
    int                 event_count;

    // Snapshot of baseline at detection time 
    float               frozen_baseline[FV_BASELINE_WINDOW];
    int                 frozen_baseline_len;
    float               frozen_threshold;   // P25 of frozen baseline 
} fv_engine_t;


// Public API

// Initialize engine (call once at startup) 
void fv_engine_init(fv_engine_t *eng);

/* Stateless compound fall rule on N scores.
 * Returns (fell, reason) where reason is a string literal. */
fv_compound_result_t fv_compound_fall_rule(const float *scores, int n);

/* Push a motion sample into the baseline ring buffer.
 * Called every cycle, regardless of state. */
void fv_baseline_push(fv_engine_t *eng, float motion_std);

/* Feed compound rule result + advance state machine.
 * now_s: current time in seconds (esp_timer_get_time() / 1e6).
 * motion_std: current cycle's std(DIAG_LT). */
void fv_engine_update(fv_engine_t *eng, float now_s,
                      bool fell, const char *reason,
                      float max_score, float motion_std);

/* Compute P-th percentile of float array (non-destructive).
 Uses insertion sort on stack copy - efficient for n <= FV_BASELINE_WINDOW. */
float fv_percentile(const float *data, int n, int percentile);

// Human-readable status string (for logging / JSON) 
const char *fv_status_str(fv_status_t status);

#endif // FALL_VALIDATION_H 
