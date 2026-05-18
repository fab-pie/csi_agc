/*
 svm_rbf.c - SVM RBF inference with float32 support vectors.
 Support vectors are stored as float32 in flash (via svm_arrays.c).
*/

#include "zoecare/rx/svm_rbf.h"
#include "zoecare/rx/svm_params.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if defined(DAPD_FAST_EXP) && DAPD_FAST_EXP
static inline float fast_expf_svm(float x)
{
    if (x < -10.0f) return 0.0f;
    if (x >  10.0f) return expf(x);
    union { float f; int32_t i; } v;
    v.i = (int32_t)(12102203.0f * x + 1065353216.0f);
    return v.f;
}
#define SVM_EXPF(x) fast_expf_svm(x)
#else
#define SVM_EXPF(x) expf(x)
#endif



// Median helper (insertion sort on small array)
static float median_small(float *buf, int n)
{
    // Insertion sort - optimal for n <= 6
    for (int i = 1; i < n; i++) {
        float key = buf[i];
        int j = i - 1;
        while (j >= 0 && buf[j] > key) {
            buf[j + 1] = buf[j];
            j--;
        }
        buf[j + 1] = key;
    }
    if (n % 2 == 1)
        return buf[n / 2];
    else
        return (buf[n / 2 - 1] + buf[n / 2]) * 0.5f;
}



/* Apply mask
 * Select SVM_N_FEATURES values from flat_input[SVM_FLAT_SIZE]
 * using precomputed mask indices.
 */
void svm_apply_mask(const float *flat_input, float *masked_out)
{
    for (int i = 0; i < SVM_MASK_SIZE; i++) {
        masked_out[i] = flat_input[svm_mask_indices[i]];
    }
}


// Pre-compute inverse scale (call once at init)
void svm_precompute_inv_scale(float *inv_out)
{
    for (int i = 0; i < SVM_N_FEATURES; i++) {
        inv_out[i] = 1.0f / svm_scaler_scale[i];
    }
}

/* StandardScaler
 * In-place: features[i] = (features[i] - mean[i]) * inv_scale[i]
 * Uses pre-computed inverse to avoid division in hot path.
 */
void svm_apply_scaler(float *features, const float *inv_scale)
{
    for (int i = 0; i < SVM_N_FEATURES; i++) {
        features[i] = (features[i] - svm_scaler_mean[i]) * inv_scale[i];
    }
}


/* SVM RBF inference (float32 support vectors)
 * decision = sum_i(alpha_i * exp(-gamma * ||x - sv_i||^2)) + intercept
 */
float calc_svm_rbf_dapd(const float *input)
{
    const float gamma = SVM_GAMMA;
    float decision_score = 0.0f;

    for (int i = 0; i < SVM_N_SV; i++) {
        // Yield to watchdog periodically
        if (i % 800 == 0 && i > 0) {
            vTaskDelay(1);
        }

        const float *sv = svm_sv_data + (size_t)i * SVM_N_FEATURES;

        // Compute ||input - sv_i||^2
        float dist_sq = 0.0f;
        for (int j = 0; j < SVM_N_FEATURES; j++) {
            float diff = input[j] - sv[j];
            dist_sq += diff * diff;
        }

        float kernel_val = SVM_EXPF(-gamma * dist_sq);
        decision_score += svm_dual_coef[i] * kernel_val;
    }

    decision_score += SVM_INTERCEPT;
    return decision_score;
}


// Prediction
int predict_svm_rbf(float decision_score, float threshold)
{
    return (decision_score > threshold) ? 1 : 0;
}



/* ── Fused transpose + median + mask ─────────────────────────────────
 * Instead of:
 *   1. transpose  smk_batch[K,4,M,n_bins] → work_buf[K,M,4*n_bins]   (45968 floats)
 *   2. median     work_buf → reduced_buf[K,N_GROUPS,4*n_bins]          (7956 floats)
 *   3. mask       reduced_buf → masked_out[634]
 *
 * Directly decode each mask index to (k, sm_type, group, bin) and
 * gather only the needed values from smk_batch in C layout.
 * This computes 634 medians instead of 7956, and eliminates both
 * intermediate buffers.
 */
void svm_fused_mask_median(const float *smk_batch,
                           int M, int n_bins, int n_groups,
                           float *masked_out)
{
    int feat_per_group = 4 * n_bins;            // 68
    int row_size = n_groups * feat_per_group;   // 612
    float tmp[8];   // scratch for median sort, max group size is 6

    for (int i = 0; i < SVM_MASK_SIZE; i++) {
        uint16_t idx = svm_mask_indices[i];

        // Decode flat index -> (k, g, sm_type, bin)
        int k         = idx / row_size;
        int remainder = idx % row_size;
        int g         = remainder / feat_per_group;
        int f         = remainder % feat_per_group;
        int sm_type   = f / n_bins;
        int bin       = f % n_bins;

        // Group boundaries (matches Python: g*M//n_groups)
        int m_start    = g * M / n_groups;
        int m_end      = (g + 1) * M / n_groups;
        int group_size = m_end - m_start;

        /* Gather values directly from smk_batch in C layout:
          smk_batch[((k*4 + sm_type) * M + m) * n_bins + bin] */
        for (int mi = 0; mi < group_size; mi++) {
            int m = m_start + mi;
            tmp[mi] = smk_batch[((k * 4 + sm_type) * M + m) * n_bins + bin];
        }

        masked_out[i] = median_small(tmp, group_size);
    }
}


// Fused pipeline
float svm_fused_pipeline(const float *smk_batch,
                         int M, int n_bins,
                         float *masked_buf,
                         const float *inv_scale)
{
    // Step 1: Fused transpose + median + mask (replaces 3 steps)
    svm_fused_mask_median(smk_batch, M, n_bins, SVM_DAPD_N_GROUPS, masked_buf);

    // Step 2: StandardScaler
    svm_apply_scaler(masked_buf, inv_scale);

    // Step 3: SVM RBF inference
    return calc_svm_rbf_dapd(masked_buf);
}
