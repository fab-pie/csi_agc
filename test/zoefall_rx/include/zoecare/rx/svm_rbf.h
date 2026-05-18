#ifndef SVM_RBF_H
#define SVM_RBF_H

#include <math.h>
#include "zoecare/rx/svm_params.h"

/* SVM RBF inference on DAPD features.
  Active pipeline: svm_fused_pipeline()
    DAPD batch output [K,4,M,n_bins]
    → fused transpose + median + mask → SVM_N_FEATURES
    → StandardScaler → RBF SVM (float32 support vectors)
 */

/* Compute SVM decision score on already-masked+scaled features.
   input: float[SVM_N_FEATURES] - scaled feature vector
   Returns: raw decision score
 */
float calc_svm_rbf_dapd(const float *input);

// Prediction from decision score
int predict_svm_rbf(float decision_score, float threshold);

// Pre-compute inverse scale: inv_out[i] = 1.0f / svm_scaler_scale[i]
void svm_precompute_inv_scale(float *inv_out);

/* Apply StandardScaler in-place: features[i] = (features[i] - mean[i]) * inv_scale[i]
   inv_scale must be pre-computed via svm_precompute_inv_scale(). */
void svm_apply_scaler(float *features, const float *inv_scale);

// Apply mask: select SVM_N_FEATURES from flat_input[SVM_FLAT_SIZE] -> masked_out */
void svm_apply_mask(const float *flat_input, float *masked_out);

/* Fused pipeline (active)
 * Fuses transpose + median + mask into a single pass.
 * Eliminates work_buf (~180 KB) and reduced_buf (~31 KB).
 * Mathematically equivalent to svm_dapd_pipeline.
 *
 * masked_buf : caller-allocated float[SVM_N_FEATURES]
 * inv_scale  : pre-computed via svm_precompute_inv_scale()
 */
float svm_fused_pipeline(const float *smk_batch,
                         int M, int n_bins,
                         float *masked_buf,
                         const float *inv_scale);

/* Fused transpose + median + mask: directly computes masked_out[634]
 * from smk_batch in C layout, skipping intermediate buffers. */
void svm_fused_mask_median(const float *smk_batch,
                           int M, int n_bins, int n_groups,
                           float *masked_out);

#endif /* SVM_RBF_H */
