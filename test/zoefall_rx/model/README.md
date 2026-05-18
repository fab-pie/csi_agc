# ZoeFall RX - SVM Model Update

This directory contains the trained SVM model and the script to deploy it to the ESP32-S3 firmware.

## Files

| File | Description |
|------|-------------|
| `*.joblib` | Trained sklearn Pipeline (StandardScaler + SVC with RBF kernel) |
| `*.npy` | Boolean feature mask - selects the active DAPD features |
| `extract_model.py` | Converts the model to C arrays for firmware embedding |
| `svm_arrays.c` / `svm_params.h` | **Auto-generated** - do not edit by hand |

## Expected Model Format

The `.joblib` file must be a `sklearn.pipeline.Pipeline` with exactly two named steps:

```python
Pipeline([
    ("standardscaler", StandardScaler()),
    ("svc", SVC(kernel="rbf")),
])
```

The `.npy` mask must be a 1D boolean array of shape `(SVM_FLAT_SIZE,)` where `mask.sum() == n_features`.

## Deploying a New Model

1. Place the new `.joblib` and `.npy` files in this directory (`model/`)
2. Run the extraction script:
   ```bash
   cd firmware/components/zoefall_rx/model
   python extract_model.py
   ```
   If there are multiple `.joblib` files, the most recently modified one is used automatically.
   You can also pass the files explicitly:
   ```bash
   python extract_model.py my_model.joblib my_mask.npy
   ```
3. Rebuild and flash the firmware:
   ```
   dev> build zoefall_rx
   dev> flash zoefall_rx:COMXX
   ```

## DAPD Parameters (must match firmware)

The model must have been trained with the following DAPD configuration:

| Parameter | Value |
|-----------|-------|
| K (windows) | 13 |
| M (subcarriers) | 52 |
| N_BINS | 17 |
| N_SM (spectral moments) | 4 |
| N_GROUPS (median reduction) | 9 |
| Flat feature size | K × M × N_SM × N_BINS = 45,968 |

**Do not change `DAPD_FAST_EXP` or `DAPD_INCREMENTAL_KDE` in the firmware** without coordinating with the firmware team - these flags affect numerical precision and must match training conditions.

## Current Model (03/04/2026)

| File | Training ID | Notes |
|------|-------------|-------|
| `3098928_0_securite584.joblib` | 3098928 | Current production model |
| `3098928_0_mask.npy` | 3098928 | Mask for current model |
