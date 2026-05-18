#ifndef ZC_ZF_DAPD_H
#define ZC_ZF_DAPD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Compile-time configuration 

// Total subcarriers
#ifndef DAPD_N_SUBCARRIERS
#define DAPD_N_SUBCARRIERS  64
#endif

// Maximum active (non-zero) subcarriers to process
#ifndef DAPD_M_ACTIVE_MAX
#define DAPD_M_ACTIVE_MAX  64
#endif

/* Maximum allowed window size Tw (number of samples per DAPD window).
  In offline mode: Tw = T - Td = T * (1 - train_ratio).
  In streaming mode: this is the fixed sliding window length. */
#ifndef DAPD_TW_MAX
#define DAPD_TW_MAX  2000
#endif

// Number of evaluation points for GKDE PDF
#ifndef DAPD_NUM_POINTS
#define DAPD_NUM_POINTS  177
#endif

// Maximum number of bins. Must be >= (int)(1/limite_seuil)
#ifndef DAPD_N_BINS
#define DAPD_N_BINS  20
#endif

// Maximum sigma (amplitude upper bound)
#ifndef DAPD_SIGMA_MAX
#define DAPD_SIGMA_MAX  400
#endif

// Profiling: Set DAPD_PROFILE=1 at compile-time to enable internal timing.
#ifndef DAPD_PROFILE
#define DAPD_PROFILE 0
#endif

#if DAPD_PROFILE
#include <stdint.h>
// Timing breakdown for one dapd_batch() call 
typedef struct {
    int64_t t_gkde_us;     // Gaussian KDE (exp loop + normaliz)
    int64_t t_binning_us;  // Peak finding + Sm1..Sm4 binning
    int64_t t_overhead_us; // signal_buf copy + loop setup + vTaskDelay
} dapd_timing_t;
#endif

// Algorithm parameters
typedef struct {
    float bandwidth;       // KDE bandwidth (default: 0.1)
    float limite_seuil;    // Linear bin width (default: 0.1 -> 10 bins)
    float alpha;           // Exponential bin scaling (default: 4.0)
    int   num_points;      // PDF evaluation points (default: DAPD_NUM_POINTS)
    float min_seuil;       // Minimum threshold (default: 0.0) 
    int   sigma;           // Amplitude upper bound (auto-detected or fixed)
    int   delta;           // Window step size (default: 1)
    float train_ratio;     // Training ratio (default: 0.9) - offline mode only
    int   use_fast_exp;    // 0 = standard expf
} dapd_params_t;

void dapd_params_default(dapd_params_t *p);

// Subcarrier map - which of the 64 subcarriers to keep

typedef struct {
    int  active[DAPD_M_ACTIVE_MAX];  // Indices of active subcarriers (0–63)
    int  count;                      // Number of active subcarriers (M)
} dapd_subcarrier_map_t;

// Build map by removing all-zero columns
int dapd_build_subcarrier_map(const float *csi_raw, int T,
                              dapd_subcarrier_map_t *map);

// Build map with all 64 subcarriers
void dapd_subcarrier_map_full(dapd_subcarrier_map_t *map);

// Build map from an explicit list of subcarrier indices
void dapd_subcarrier_map_from_list(const int *indices, int count,
                                   dapd_subcarrier_map_t *map);

// GKDE - Gaussian Kernel Density Estimation

// Compute GKDE PDF of signal[0..T-1]
void dapd_gkde_pdf(const float *signal, int T,
                   int sigma, float bandwidth,
                   int num_points, int use_fast_exp,
                   float *pdf, float *s_axis);

// Single-window DAPD computation
void dapd_compute_window(const float *window_data, int Tw,
                         const dapd_params_t *params,
                         const float *exp_bins_precomp,
                         int M,
                         float *smk_out);

// Streaming (ring-buffer) interface, for ESP32 real-time use

typedef struct {
    float   ring_buf[DAPD_TW_MAX * DAPD_M_ACTIVE_MAX];
    int     head;           // Next write position (0..Tw-1)
    int     count;          // Samples accumulated (0..Tw)
    int     Tw;             // Window size (fixed at init)
    int     M;              // Active subcarriers

    dapd_params_t params;
    dapd_subcarrier_map_t map;

    // Output buffer: latest Smk [4 * M * n_bins]
    float   smk[4 * DAPD_M_ACTIVE_MAX * DAPD_N_BINS];

    // Internal scratch (avoid stack allocation)
    float   pdf_buf[DAPD_NUM_POINTS];
    float   s_axis_buf[DAPD_NUM_POINTS];
    float   win_extract[DAPD_TW_MAX];

    // Exponential bins (precomputed)
    float   exp_bins[DAPD_N_BINS + 1];

    // Stats
    int     windows_computed;
    int     samples_received;
} dapd_stream_t;

// Initialize streaming state
int dapd_stream_init(dapd_stream_t *st,
                     int Tw,
                     const dapd_subcarrier_map_t *map,
                     const dapd_params_t *params);

// Push one AMP sample (64 raw amplitudes) into the ring buffer
int dapd_stream_push(dapd_stream_t *st,
                     const int16_t raw_amps[DAPD_N_SUBCARRIERS]);

// Push float amplitudes (extracted/scaled)
int dapd_stream_push_float(dapd_stream_t *st,
                           const float *amps_active, int M);

// Compute DAPD on the current window
const float *dapd_stream_compute(dapd_stream_t *st);

// Reset stream state (clear buffer, keep params) */
void dapd_stream_reset(dapd_stream_t *st);

// Offline batch interface. Only for PC execution
void dapd_batch(const float *csi, int T, int M,
                int Td, int Tw, int K,
                const dapd_params_t *params,
                const float *exp_bins_precomp,
                float *smk_out,
                int *K_out, int *n_bins_out
#if DAPD_PROFILE
                , dapd_timing_t *timing
#endif
                );

#ifdef __cplusplus
}
#endif

#endif
