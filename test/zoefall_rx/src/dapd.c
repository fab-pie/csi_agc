#include "zoecare/rx/dapd.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if DAPD_PROFILE
#include "esp_timer.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Flags check in CMakeLists.txt
#ifndef DAPD_FAST_EXP
    #error "DAPD_FAST_EXP not defined in CMakeLists.txt"
#endif

#ifndef DAPD_TW_MAX
    #error "DAPD_TW_MAX not defined in CMakeLists.txt"
#endif

#ifndef DAPD_M_ACTIVE_MAX
    #error "DAPD_M_ACTIVE_MAX not defined in CMakeLists.txt"
#endif

#ifndef DAPD_NUM_POINTS
    #error "DAPD_NUM_POINTS not defined in CMakeLists.txt"
#endif

#ifndef DAPD_INCREMENTAL_KDE
    #define DAPD_INCREMENTAL_KDE 0
#endif

// -------------------------------------------------------------------
#pragma region HELPERS
// -------------------------------------------------------------------

// Experiments on speeding up exponential function for PDF creation starting from CSI samples,
// All the different tries gave unsatisfactory results, so actually we apply Binned KDE via Convolution

// Fast exp approximation for ESP32 with Schraudolph's method, empirically not valid so unused
static inline float fast_expf(float x)
{
    // Clamp to avoid overflow/underflow in integer cast
    if (x < -10.0f) return 0.0f;
    if (x >  10.0f) return expf(x); // fallback for very large
    union { float f; int32_t i; } v;
    v.i = (int32_t)(12102203.0f * x + 1065353216.0f);
    return v.f;
}

// Define Look Up Table, do it only once at boot
#define LUT_SIZE 4096
#define LUT_MAX_Y 8.0f

#if DAPD_FAST_EXP == 2
float exp_lut[LUT_SIZE];  // 16 KB - only allocated when LUT mode is active

void init_exp_lut() {
    for (int i = 0; i < LUT_SIZE; i++) {
        float y = i * (LUT_MAX_Y / (LUT_SIZE - 1));
        exp_lut[i] = expf(-y);
    }
}

// LUT-based exp approximation
static inline float exact_fast_exp_neg(float y) {
    if (y >= LUT_MAX_Y) return 0.0f;
    if (y < 0.0f) return expf(-y);

    float float_idx = y * ((LUT_SIZE - 1) / LUT_MAX_Y);
    int idx = (int)float_idx;
    float frac = float_idx - idx;

    // Linear interpolation
    return exp_lut[idx] + frac * (exp_lut[idx + 1] - exp_lut[idx]);
}
#else
void init_exp_lut(void) { /* no-op when LUT mode is not active */ }
#endif /* DAPD_FAST_EXP == 2 */

// Fast exp approximation with Horner form
static inline float fast_exp_horner(float x) {
    // Horner form: c0 + x*(c1 + x*(c2 + x*(c3 + x*(c4 + x*(c5 + x*c6)))))
    return 1.0f + x*(1.0000001f + x*(0.4999966f + x*(0.1666760f
                + x*(0.0416573f + x*(0.0083357f + x*0.0013889f)))));
}

#if DAPD_FAST_EXP == 1
    #define EXPF(x) fast_expf(x)
#elif DAPD_FAST_EXP == 2
    #define EXPF(x) ((x) <= 0 ? exact_fast_exp_neg(-(x)) : expf(x))
#elif DAPD_FAST_EXP == 3
    #define EXPF(x) ((x) <= 0 ? fast_exp_horner(x) : expf(x))
#else
    #define EXPF(x) expf(x)
#endif

// Trapezoidal integration (used for PDF normalization)
static float trapezoid_f(const float *y, const float *x, int n)
{
    float area = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        area += 0.5f * (y[i] + y[i + 1]) * (x[i + 1] - x[i]);
    }
    return area;
}

// Binary search for digitize_right (equivalent to np.digitize(x, bins, right=True))
static int digitize_right_f(float x, const float *bins, int n_bins_len)
{
    int lo = 0, hi = n_bins_len;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (bins[mid] < x)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

void dapd_params_default(dapd_params_t *p)
{
    p->bandwidth    = 0.5f;
    p->limite_seuil = 0.056f;
    p->alpha        = 4.0f;
    p->num_points   = 177;
    p->min_seuil    = 0.0f;
    p->sigma        = 0;       // Set by caller or auto-detected
    p->delta        = 25;
    p->train_ratio  = 0.9f;
    p->use_fast_exp = DAPD_FAST_EXP;      
    //TD 335   Tw= 400-TD
}

// -------------------------------------------------------------------
#pragma region SUBCARRIER MAP
// -------------------------------------------------------------------

// Build map of nonzero subcarriers
int dapd_build_subcarrier_map(const float *csi_raw, int T,
                              dapd_subcarrier_map_t *map)
{
    map->count = 0;
    for (int m = 0; m < DAPD_N_SUBCARRIERS && map->count < DAPD_M_ACTIVE_MAX; m++) {
        int nonzero = 0;
        for (int t = 0; t < T; t++) {
            if (csi_raw[t * DAPD_N_SUBCARRIERS + m] != 0.0f) {
                nonzero = 1;
                break;
            }
        }
        if (nonzero) {
            map->active[map->count++] = m;
        }
    }
    return map->count;
}

// Build map of all subcarriers (also null ones), for debug or test, not actually used in preprocessing
void dapd_subcarrier_map_full(dapd_subcarrier_map_t *map)
{
    map->count = DAPD_N_SUBCARRIERS;
    for (int i = 0; i < DAPD_N_SUBCARRIERS; i++)
        map->active[i] = i;
}

// Build map of subcarriers given their indices
void dapd_subcarrier_map_from_list(const int *indices, int count,
                                   dapd_subcarrier_map_t *map)
{
    if (count > DAPD_M_ACTIVE_MAX) count = DAPD_M_ACTIVE_MAX;
    map->count = count;
    for (int i = 0; i < count; i++)
        map->active[i] = indices[i];
}

// -------------------------------------------------------------------
#pragma region GKDE PDF
// -------------------------------------------------------------------

void dapd_gkde_pdf(const float *signal, int T,
                   int sigma, float bandwidth,
                   int num_points, int use_fast_exp,
                   float *pdf, float *s_axis)
{

    // build x axis, amplitude scale
    float step = (num_points > 1)
        ? (float)sigma / (float)(num_points - 1)
        : 0.0f;
    for (int i = 0; i < num_points; i++) {
        s_axis[i] = i * step;
    }

    float inv_bw = 1.0f / bandwidth;
    float norm_factor = 1.0f / (sqrtf(2.0f * (float)M_PI) * bandwidth);

    float cutoff = 4.0f;
    float cutoff_val = cutoff * bandwidth;  // radius in signal space

    float inv_step = (step > 0.0f) ? 1.0f / step : 0.0f;
    float step_inv_bw = step * inv_bw;

    memset(pdf, 0, num_points * sizeof(float));

    // Create a gaussian curve for every amplitude value (that is the center of the curve)
    // at each time instant t and sum it to the others to get the PDF of the whole Tw
    for (int j = 0; j < T; j++) {
        float sig_j = signal[j];
        int i_lo = (int)((sig_j - cutoff_val) * inv_step);
        int i_hi = (int)((sig_j + cutoff_val) * inv_step) + 1;
        if (i_lo < 0) i_lo = 0;
        if (i_hi > num_points) i_hi = num_points;

        float diff_start = (s_axis[i_lo] - sig_j) * inv_bw;
        float diff = diff_start;

        // Choose exponential computation based on what is set in CMakeLists.txt
        if (use_fast_exp == 3) {
            // MODE 3: Fast exp Horner
            for (int i = i_lo; i < i_hi; i++) {
                float y = -0.5f * diff * diff;
                pdf[i] += fast_exp_horner(y);
                diff += step_inv_bw;
            }
        }


#if DAPD_FAST_EXP == 2
        else if (use_fast_exp == 2) {
            // MODE 2: Look-Up Table
            for (int i = i_lo; i < i_hi; i++) {
                float y = 0.5f * diff * diff;
                pdf[i] += exact_fast_exp_neg(y);
                diff += step_inv_bw;
            }
        }
#endif
        else if (use_fast_exp == 1) {
            // MODE 1: Schraudolph's method
            for (int i = i_lo; i < i_hi; i++) {
                float y = -0.5f * diff * diff;
                pdf[i] += fast_expf(y);
                diff += step_inv_bw;
            }
        } 
        else {
            // MODE 0: Standard expf
            for (int i = i_lo; i < i_hi; i++) {
                float y = -0.5f * diff * diff;
                pdf[i] += expf(y);
                diff += step_inv_bw;
            }
        }
    }

    for (int i = 0; i < num_points; i++) {
        pdf[i] *= norm_factor;
    }

    // Trapezoidal normalization
    float area = trapezoid_f(pdf, s_axis, num_points);
    if (area > 0.0f) {
        float inv_area = 1.0f / area;
        for (int i = 0; i < num_points; i++) {
            pdf[i] *= inv_area;
        }
    }
}


// Binned KDE via Convolution
void dapd_gkde_pdf_BKC(const float *signal, int T,
                   int sigma, float bandwidth,
                   int num_points, int use_fast_exp,
                   float *pdf, float *s_axis)
{
    float step = (num_points > 1) ? (float)sigma / (float)(num_points - 1) : 0.0f;
    for (int i = 0; i < num_points; i++) {
        s_axis[i] = i * step;
    }

    memset(pdf, 0, num_points * sizeof(float));

    if (step <= 0.0f) {
        return; // Prevent dividing for 0
    }

    float inv_step = 1.0f / step;
    float inv_bw = 1.0f / bandwidth;
    float norm_factor = 1.0f / (sqrtf(2.0f * (float)M_PI) * bandwidth);

    // Linear binning
    float grid[DAPD_NUM_POINTS] = {0}; 

    for (int j = 0; j < T; j++) {
        float f_idx = signal[j] * inv_step;
        int idx = (int)f_idx;               // parte intera
        float frac = f_idx - (float)idx;    // parte frazionaria

        if (idx < 0) {
            grid[0] += 1.0f; // underflow: clamp to first bin (mirrors dapd_gkde_pdf i_lo=0)
        } else if (idx >= num_points) {
            grid[num_points - 1] += 1.0f; // overflow: clamp to last bin
        } else if (idx == num_points - 1) {
            grid[idx] += 1.0f;
        } else {
            grid[idx]     += (1.0f - frac);
            grid[idx + 1] += frac;
        }
    }

    // Pre computation of kernel
    float cutoff_val = 5.0f * bandwidth;
    int k_radius = (int)(cutoff_val * inv_step) + 1;
    if (k_radius >= num_points) k_radius = num_points - 1;

    int k_size = 2 * k_radius + 1;
    float kernel[DAPD_NUM_POINTS * 2 + 1]; 
    
    for (int i = -k_radius; i <= k_radius; i++) {
        float diff = (i * step) * inv_bw;
        float y = -0.5f * diff * diff;
        
        if (use_fast_exp == 3)      kernel[i + k_radius] = fast_exp_horner(y);
#if DAPD_FAST_EXP == 2
        else if (use_fast_exp == 2) kernel[i + k_radius] = exact_fast_exp_neg(-y);
#endif
        else if (use_fast_exp == 1) kernel[i + k_radius] = fast_expf(y);
        else                        kernel[i + k_radius] = expf(y);
    }

    // Scatter convolution (multiply resulting histogram with gaussian kernel)
    for (int i = 0; i < num_points; i++) {
        if (grid[i] == 0.0f) continue; 

        float weight = grid[i];
        
        int j_start = i - k_radius;
        int j_end   = i + k_radius;
        int k_start = 0;

        if (j_start < 0) {
            k_start += (0 - j_start);
            j_start = 0;
        }
        if (j_end >= num_points) {
            j_end = num_points - 1;
        }

        for (int j = j_start, k = k_start; j <= j_end; j++, k++) {
            pdf[j] += weight * kernel[k];
        }
    }

    // Normalize
    for (int i = 0; i < num_points; i++) {
        pdf[i] *= norm_factor;
    }

    float area = trapezoid_f(pdf, s_axis, num_points);
    if (area > 0.0f) {
        float inv_area = 1.0f / area;
        for (int i = 0; i < num_points; i++) {
            pdf[i] *= inv_area;
        }
    }
}
// -------------------------------------------------------------------
#pragma region INCREMENTAL KDE HELPERS
// -------------------------------------------------------------------

// Incremental KDE is actually unused, the results on svm were very unsatisfactory both
// on time saving and quality of the final computation (too wide difference wrt no incremental kde)

// Add one sample's Gaussian kernel contribution to an un-normalized raw KDE accumulator.
static void kde_add_sample(float *pdf_raw, float sig_j,
                            const float *s_axis, int num_points,
                            float bandwidth, int use_fast_exp)
{
    float inv_bw      = 1.0f / bandwidth;
    float step        = (num_points > 1) ? s_axis[1] - s_axis[0] : 0.0f;
    float inv_step    = (step > 0.0f) ? 1.0f / step : 0.0f;
    float cutoff_val  = 4.0f * bandwidth;
    float step_inv_bw = step * inv_bw;

    int i_lo = (int)((sig_j - cutoff_val) * inv_step);
    int i_hi = (int)((sig_j + cutoff_val) * inv_step) + 1;
    if (i_lo < 0)          i_lo = 0;
    if (i_hi > num_points) i_hi = num_points;

    float diff = (s_axis[i_lo] - sig_j) * inv_bw;
    for (int i = i_lo; i < i_hi; i++) {
        float y = -0.5f * diff * diff;
        pdf_raw[i] += EXPF(y);
        diff += step_inv_bw;
    }
    (void)use_fast_exp;
}

// Subtract one sample's Gaussian kernel contribution from an un-normalized raw KDE accumulator.
static void kde_sub_sample(float *pdf_raw, float sig_j,
                            const float *s_axis, int num_points,
                            float bandwidth, int use_fast_exp)
{
    float inv_bw      = 1.0f / bandwidth;
    float step        = (num_points > 1) ? s_axis[1] - s_axis[0] : 0.0f;
    float inv_step    = (step > 0.0f) ? 1.0f / step : 0.0f;
    float cutoff_val  = 4.0f * bandwidth;
    float step_inv_bw = step * inv_bw;

    int i_lo = (int)((sig_j - cutoff_val) * inv_step);
    int i_hi = (int)((sig_j + cutoff_val) * inv_step) + 1;
    if (i_lo < 0)          i_lo = 0;
    if (i_hi > num_points) i_hi = num_points;

    float diff = (s_axis[i_lo] - sig_j) * inv_bw;
    for (int i = i_lo; i < i_hi; i++) {
        float y = -0.5f * diff * diff;
        pdf_raw[i] -= EXPF(y);
        diff += step_inv_bw;
    }
    (void)use_fast_exp;
}

// Normalize a raw KDE accumulator into a proper unit-area PDF.
static void kde_finalize(const float *pdf_raw, const float *s_axis,
                          int num_points, float bandwidth, int n_samples,
                          float *pdf_out)
{
    (void)n_samples;
    float norm_factor = 1.0f / (sqrtf(2.0f * (float)M_PI) * bandwidth);
    for (int i = 0; i < num_points; i++)
        pdf_out[i] = pdf_raw[i] * norm_factor;

    float area = trapezoid_f(pdf_out, s_axis, num_points);
    if (area > 0.0f) {
        float inv_area = 1.0f / area;
        for (int i = 0; i < num_points; i++)
            pdf_out[i] *= inv_area;
    }
}

// -------------------------------------------------------------------
#pragma region DAPD WINDOW
// -------------------------------------------------------------------

static void compute_sm_one_subcarrier(
    const float *signal, int Tw,
    const dapd_params_t *params,
    const float *exp_bins,
    float *pdf, float *s_axis,
    float *Sm1, float *Sm2, float *Sm3, float *Sm4
#if DAPD_PROFILE
    , int64_t *t_gkde_acc, int64_t *t_binning_acc
#endif
    )
{
    int n_bins = (int)(1.0f / params->limite_seuil);
    float min_seuil = params->min_seuil;
    float limite_seuil = params->limite_seuil;

#if DAPD_PROFILE
    int64_t _t0_kde = esp_timer_get_time();
#endif
#if BINNED_KDE_CONV
    dapd_gkde_pdf_BKC(signal, Tw, params->sigma, params->bandwidth,
                  params->num_points, params->use_fast_exp, pdf, s_axis);
#else
    dapd_gkde_pdf(signal, Tw, params->sigma, params->bandwidth,
                  params->num_points, params->use_fast_exp, pdf, s_axis);
#endif
#if DAPD_PROFILE
    if (t_gkde_acc) *t_gkde_acc += esp_timer_get_time() - _t0_kde;
    int64_t _t0_bin = esp_timer_get_time();
#endif

    // Find peak index
    int peak_idx = 0;
    for (int i = 1; i < params->num_points; i++) {
        if (pdf[i] > pdf[peak_idx]) peak_idx = i;
    }

    // Cut the PDF in two halves separated by the peak
    int f1_len = peak_idx + 1;
    int f2_len = params->num_points - f1_len;
    float *f1 = pdf;
    float *f2 = pdf + f1_len;

    for (int b = 0; b < n_bins; b++)
        Sm1[b] = Sm2[b] = Sm3[b] = Sm4[b] = 0.0f;

    // Normalize by Tw
    float inv_Tw = (Tw > 0) ? 1.0f / (float)Tw : 0.0f;

    // Sm1: linear binning of f1, REVERSED
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f1_len; i++) {
            if (f1[i] <= min_seuil) continue;
            int idx = (int)((f1[i] - min_seuil) / limite_seuil);
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm1[n_bins - 1 - b] = (float)counts[b] * inv_Tw;
    }

    // Sm2: linear binning of f2, normal
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f2_len; i++) {
            if (f2[i] <= min_seuil) continue;
            int idx = (int)((f2[i] - min_seuil) / limite_seuil);
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm2[b] = (float)counts[b] * inv_Tw;
    }

    // Sm3: exponential binning of f1, REVERSED
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f1_len; i++) {
            int idx = digitize_right_f(f1[i], exp_bins, n_bins + 1) - 1;
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm3[n_bins - 1 - b] = (float)counts[b] * inv_Tw;
    }

    // Sm4: exponential binning of f2, normal
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f2_len; i++) {
            int idx = digitize_right_f(f2[i], exp_bins, n_bins + 1) - 1;
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm4[b] = (float)counts[b] * inv_Tw;
    }
#if DAPD_PROFILE
    if (t_binning_acc) *t_binning_acc += esp_timer_get_time() - _t0_bin;
#endif
}

// Redefine function to do only binning (when PDF is already computed, only in option DAPD_INCREMENTAL_KDE=1)
static void compute_sm_from_pdf(
    const float *pdf, int num_points,
    const dapd_params_t *params,
    const float *exp_bins,
    int Tw,
    float *Sm1, float *Sm2, float *Sm3, float *Sm4)
{
    int n_bins = (int)(1.0f / params->limite_seuil);
    float min_seuil    = params->min_seuil;
    float limite_seuil = params->limite_seuil;

    // Find peak index
    int peak_idx = 0;
    for (int i = 1; i < num_points; i++) {
        if (pdf[i] > pdf[peak_idx]) peak_idx = i;
    }

    int f1_len    = peak_idx + 1;
    int f2_len    = num_points - f1_len;
    const float *f1 = pdf;
    const float *f2 = pdf + f1_len;

    for (int b = 0; b < n_bins; b++)
        Sm1[b] = Sm2[b] = Sm3[b] = Sm4[b] = 0.0f;

    float inv_Tw = (Tw > 0) ? 1.0f / (float)Tw : 0.0f;

    // Sm1: linear binning of f1, REVERSED
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f1_len; i++) {
            if (f1[i] <= min_seuil) continue;
            int idx = (int)((f1[i] - min_seuil) / limite_seuil);
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm1[n_bins - 1 - b] = (float)counts[b] * inv_Tw;
    }

    // Sm2: linear binning of f2, normal
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f2_len; i++) {
            if (f2[i] <= min_seuil) continue;
            int idx = (int)((f2[i] - min_seuil) / limite_seuil);
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm2[b] = (float)counts[b] * inv_Tw;
    }

    // Sm3: exponential binning of f1, REVERSED
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f1_len; i++) {
            int idx = digitize_right_f(f1[i], exp_bins, n_bins + 1) - 1;
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm3[n_bins - 1 - b] = (float)counts[b] * inv_Tw;
    }

    // Sm4: exponential binning of f2, normal
    {
        int counts[DAPD_N_BINS];
        memset(counts, 0, n_bins * sizeof(int));
        for (int i = 0; i < f2_len; i++) {
            int idx = digitize_right_f(f2[i], exp_bins, n_bins + 1) - 1;
            if (idx >= 0 && idx < n_bins) counts[idx]++;
        }
        for (int b = 0; b < n_bins; b++)
            Sm4[b] = (float)counts[b] * inv_Tw;
    }
}

// Wrapper function to handle Tw
void dapd_compute_window(const float *window_data, int Tw,
                         const dapd_params_t *params,
                         const float *exp_bins_precomp,
                         int M,
                         float *smk_out)
{
    int n_bins = (int)(1.0f / params->limite_seuil);

    // Static buffers to avoid ~10KB stack allocation
    // Trade-off: not thread-safe. Use dapd_stream_compute for concurrent use.
    static float pdf[DAPD_NUM_POINTS];
    static float s_axis[DAPD_NUM_POINTS];
    static float signal[DAPD_TW_MAX];

    for (int m = 0; m < M; m++) {
        // Extract column m from window_data
        for (int t = 0; t < Tw; t++) {
            signal[t] = window_data[t * M + m];
        }

        float *Sm1 = smk_out + (0 * M + m) * n_bins;
        float *Sm2 = smk_out + (1 * M + m) * n_bins;
        float *Sm3 = smk_out + (2 * M + m) * n_bins;
        float *Sm4 = smk_out + (3 * M + m) * n_bins;

        compute_sm_one_subcarrier(signal, Tw, params, exp_bins_precomp,
                                  pdf, s_axis, Sm1, Sm2, Sm3, Sm4
#if DAPD_PROFILE
                                  , NULL, NULL
#endif
                                  );
    }
}

// -------------------------------------------------------------------
#pragma region STREAMING
// -------------------------------------------------------------------

int dapd_stream_init(dapd_stream_t *st,
                     int Tw,
                     const dapd_subcarrier_map_t *map,
                     const dapd_params_t *params)
{
    if (Tw > DAPD_TW_MAX || Tw <= 0) {
        fprintf(stderr, "dapd_stream_init: Tw=%d exceeds DAPD_TW_MAX=%d\n",
                Tw, DAPD_TW_MAX);
        return -1;
    }
    if (map->count > DAPD_M_ACTIVE_MAX || map->count <= 0) {
        fprintf(stderr, "dapd_stream_init: M=%d exceeds DAPD_M_ACTIVE_MAX=%d\n",
                map->count, DAPD_M_ACTIVE_MAX);
        return -1;
    }
    if (params->sigma <= 0) {
        fprintf(stderr, "dapd_stream_init: sigma must be > 0 (got %d)\n",
                params->sigma);
        return -1;
    }

    memset(st, 0, sizeof(dapd_stream_t));
    st->Tw = Tw;
    st->M  = map->count;
    st->head = 0;
    st->count = 0;
    st->windows_computed = 0;
    st->samples_received = 0;

    memcpy(&st->params, params, sizeof(dapd_params_t));
    memcpy(&st->map, map, sizeof(dapd_subcarrier_map_t));

    // Precompute exponential bins
    int n_bins = (int)(1.0f / params->limite_seuil);
    float exp_alpha = EXPF(params->alpha);
    for (int i = 0; i <= n_bins; i++) {
        float t = (float)i / (float)n_bins;
        st->exp_bins[i] = params->min_seuil
            + (1.0f - params->min_seuil)
            * (EXPF(params->alpha * t) - 1.0f) / (exp_alpha - 1.0f);
    }

    return 0;
}

int dapd_stream_push(dapd_stream_t *st,
                     const int16_t raw_amps[DAPD_N_SUBCARRIERS])
{
    int M = st->M;
    int row_offset = st->head * M;

    // Extract only active subcarriers
    for (int i = 0; i < M; i++) {
        st->ring_buf[row_offset + i] = (float)raw_amps[st->map.active[i]];
    }

    st->head = (st->head + 1) % st->Tw;
    st->samples_received++;
    if (st->count < st->Tw) st->count++;

    return (st->count >= st->Tw) ? 1 : 0;
}

int dapd_stream_push_float(dapd_stream_t *st,
                           const float *amps_active, int M)
{
    if (M != st->M) return -1;

    int row_offset = st->head * M;
    memcpy(&st->ring_buf[row_offset], amps_active, M * sizeof(float));

    st->head = (st->head + 1) % st->Tw;
    st->samples_received++;
    if (st->count < st->Tw) st->count++;

    return (st->count >= st->Tw) ? 1 : 0;
}

const float *dapd_stream_compute(dapd_stream_t *st)
{
    if (st->count < st->Tw) return NULL;

    int Tw = st->Tw;
    int M = st->M;
    int n_bins = (int)(1.0f / st->params.limite_seuil);

    // Zero output
    memset(st->smk, 0, 4 * M * n_bins * sizeof(float));

    // Process each active subcarrier
    for (int m = 0; m < M; m++) {
        //  Linearize ring buffer column m into win_extract
        int read_pos = st->head; // head points to oldest sample
        for (int t = 0; t < Tw; t++) {
            st->win_extract[t] = st->ring_buf[read_pos * M + m];
            read_pos = (read_pos + 1) % Tw;
        }

        float *Sm1 = st->smk + (0 * M + m) * n_bins;
        float *Sm2 = st->smk + (1 * M + m) * n_bins;
        float *Sm3 = st->smk + (2 * M + m) * n_bins;
        float *Sm4 = st->smk + (3 * M + m) * n_bins;

        compute_sm_one_subcarrier(st->win_extract, Tw,
                                  &st->params, st->exp_bins,
                                  st->pdf_buf, st->s_axis_buf,
                                  Sm1, Sm2, Sm3, Sm4
#if DAPD_PROFILE
                                  , NULL, NULL
#endif
                                  );
    }

    st->windows_computed++;
    return st->smk;
}

void dapd_stream_reset(dapd_stream_t *st)
{
    st->head = 0;
    st->count = 0;
    st->windows_computed = 0;
    st->samples_received = 0;
    memset(st->ring_buf, 0, st->Tw * st->M * sizeof(float));
    memset(st->smk, 0, 4 * st->M * DAPD_N_BINS * sizeof(float));
}

// -------------------------------------------------------------------
#pragma region BATCH
// -------------------------------------------------------------------

void dapd_batch(const float *csi, int T, int M,
                int Td, int Tw, int K,
                const dapd_params_t *params,
                const float *exp_bins_precomp,
                float *smk_out,
                int *K_out, int *n_bins_out
#if DAPD_PROFILE
                , dapd_timing_t *timing
#endif
                )
{
    int n_bins = (int)(1.0f / params->limite_seuil);
    int delta  = params->delta;

    *K_out     = K;
    *n_bins_out = n_bins;

#if DAPD_PROFILE
    if (timing) { timing->t_gkde_us = 0; timing->t_binning_us = 0; timing->t_overhead_us = 0; }
    int64_t _t_batch_start = esp_timer_get_time();
#endif

#if DAPD_INCREMENTAL_KDE
    // Incremental (sliding) KDE path
    int num_points = params->num_points;
    float pdf_raw[DAPD_NUM_POINTS];
    float pdf_fin[DAPD_NUM_POINTS];
    float s_axis[DAPD_NUM_POINTS];

    {
        float step = (num_points > 1) ? (float)params->sigma / (float)(num_points - 1) : 0.0f;
        for (int i = 0; i < num_points; i++)
            s_axis[i] = i * step;
    }

#if DAPD_PROFILE
    int64_t _t_gkde = 0, _t_binning = 0;
#endif

    for (int m = 0; m < M; m++) {
        if (m == 0) vTaskDelay(1);

        // Window k=1: full KDE build
        int win0_start = T - Td - Tw + delta;

#if DAPD_PROFILE
        int64_t _t0 = esp_timer_get_time();
#endif
        memset(pdf_raw, 0, num_points * sizeof(float));
        for (int t = 0; t < Tw; t++)
            kde_add_sample(pdf_raw, csi[(win0_start + t) * M + m],
                           s_axis, num_points, params->bandwidth, params->use_fast_exp);
        kde_finalize(pdf_raw, s_axis, num_points, params->bandwidth, Tw, pdf_fin);
#if DAPD_PROFILE
        _t_gkde += esp_timer_get_time() - _t0;
        _t0 = esp_timer_get_time();
#endif
        {
            float *Sm1 = smk_out + ((0 + 0) * M + m) * n_bins;
            float *Sm2 = smk_out + ((0 + 1) * M + m) * n_bins;
            float *Sm3 = smk_out + ((0 + 2) * M + m) * n_bins;
            float *Sm4 = smk_out + ((0 + 3) * M + m) * n_bins;
            compute_sm_from_pdf(pdf_fin, num_points, params, exp_bins_precomp, Tw,
                                Sm1, Sm2, Sm3, Sm4);
        }
#if DAPD_PROFILE
        _t_binning += esp_timer_get_time() - _t0;
#endif

        // Windows k=2..K: incremental slide
        for (int k = 2; k <= K; k++) {
            if (m == 0) vTaskDelay(1);

#if DAPD_PROFILE
            _t0 = esp_timer_get_time();
#endif
            int prev_win_start = T - Td - Tw + (k - 1) * delta;
            int new_win_end    = prev_win_start + Tw;
            for (int i = 0; i < delta; i++)
                kde_sub_sample(pdf_raw, csi[(prev_win_start + i) * M + m],
                               s_axis, num_points, params->bandwidth, params->use_fast_exp);
            for (int i = 0; i < delta; i++)
                kde_add_sample(pdf_raw, csi[(new_win_end + i) * M + m],
                               s_axis, num_points, params->bandwidth, params->use_fast_exp);
            kde_finalize(pdf_raw, s_axis, num_points, params->bandwidth, Tw, pdf_fin);
#if DAPD_PROFILE
            _t_gkde += esp_timer_get_time() - _t0;
            _t0 = esp_timer_get_time();
#endif
            int base_k = (k - 1) * 4;
            float *Sm1 = smk_out + ((base_k + 0) * M + m) * n_bins;
            float *Sm2 = smk_out + ((base_k + 1) * M + m) * n_bins;
            float *Sm3 = smk_out + ((base_k + 2) * M + m) * n_bins;
            float *Sm4 = smk_out + ((base_k + 3) * M + m) * n_bins;
            compute_sm_from_pdf(pdf_fin, num_points, params, exp_bins_precomp, Tw,
                                Sm1, Sm2, Sm3, Sm4);
#if DAPD_PROFILE
            _t_binning += esp_timer_get_time() - _t0;
#endif
        }
    }

#if DAPD_PROFILE
    if (timing) {
        int64_t _t_batch_total = esp_timer_get_time() - _t_batch_start;
        timing->t_gkde_us    = _t_gkde;
        timing->t_binning_us = _t_binning;
        timing->t_overhead_us = _t_batch_total - _t_gkde - _t_binning;
    }
#endif

#else
    // Standard (non-incremental) path
    float pdf[DAPD_NUM_POINTS];
    float s_axis[DAPD_NUM_POINTS];
    float signal_buf[DAPD_TW_MAX];
#if DAPD_PROFILE
    int64_t _t_gkde   = 0;
    int64_t _t_binning = 0;
#endif

    for (int k = 1; k <= K; k++) {
        if (k % 10 == 0) vTaskDelay(1);
        int kdelta    = k * delta;
        int win_start = T - Td - Tw + kdelta;
        int win_len   = Tw;
        int base_k    = (k - 1) * 4;

        for (int m = 0; m < M; m++) {
            for (int t = 0; t < win_len; t++)
                signal_buf[t] = csi[(win_start + t) * M + m];

            float *Sm1 = smk_out + ((base_k + 0) * M + m) * n_bins;
            float *Sm2 = smk_out + ((base_k + 1) * M + m) * n_bins;
            float *Sm3 = smk_out + ((base_k + 2) * M + m) * n_bins;
            float *Sm4 = smk_out + ((base_k + 3) * M + m) * n_bins;

            compute_sm_one_subcarrier(signal_buf, win_len, params, exp_bins_precomp,
                                      pdf, s_axis, Sm1, Sm2, Sm3, Sm4
#if DAPD_PROFILE
                                      , &_t_gkde, &_t_binning
#endif
                                      );
        }

        if (k % 50 == 0 || k == K)
            fprintf(stderr, "  Window k=%d/%d done\n", k, K);
    }
#if DAPD_PROFILE
    if (timing) {
        int64_t _t_batch_total = esp_timer_get_time() - _t_batch_start;
        timing->t_gkde_us    = _t_gkde;
        timing->t_binning_us = _t_binning;
        timing->t_overhead_us = _t_batch_total - _t_gkde - _t_binning;
    }
#endif
#endif
}
