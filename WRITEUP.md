# WRITEUP — Changes to Support CAN-Decoded Dataset

**Date:** 2026-03-31  
**Author:** Cline (AI-assisted)

---

## Problem

The EKF pipeline was originally written for a single CSV format (`localization_log2.csv`) with columns like `gps_lat`, `gps_lon`, `gps_speed`, `imu_yaw`, `imu_yaw_rate`, etc. When running with the new CAN-decoded dataset (`datasets/can_log_vcu_*.csv`), the results were completely wrong because:

1. **Column layout mismatch** — The new CSV has different column names and positions (e.g., `0x400_Latitude` at column 12 instead of `gps_lat` at column 2).
2. **Unit differences** — GPS coordinates are scaled by 10⁷, velocity is in mm/s, and gyro Z is in deg/s.
3. **Missing fields** — The yaw angle column (`0x388_Y`) is always empty in the new dataset.
4. **Division by zero** — When yaw rate is exactly 0 (common in the new dataset), the EKF propagation step computed `r = v / omega = v / 0 = inf`, corrupting the entire state with NaN.

---

## Changes Made

### 1. `main_utm_2.cpp` — Auto-Detect CSV Format from Header

**What changed:**  
Replaced hardcoded column indices with a header-based auto-detection system.

**How it works:**  
On startup, the code reads the CSV header line and looks for known column names to determine the format:

| Detection Rule | Format |
|---|---|
| Header contains `gps_lat` and `gps_lon` | `FORMAT_OLD` (localization_log2 style) |
| Header contains `0x400_Latitude` or `0x400_Logitude` | `FORMAT_CAN` (CAN-decoded style) |
| Neither matches | Falls back to old-style indices with a warning |

A `ColumnMap` struct stores the detected column indices and unit conversion factors:

```
struct ColumnMap {
    CsvFormat format;
    int col_lat, col_lon;           // GPS position columns
    int col_speed;                  // Forward velocity column
    int col_yaw;                    // Yaw angle column (-1 if unavailable)
    int col_yaw_rate;               // Yaw rate column
    int col_qw, col_qx, col_qy, col_qz;  // Quaternion columns (-1 if unavailable)
    double lat_scale, lon_scale;    // Raw value × scale = degrees
    double speed_scale;             // Raw value × scale = m/s
    double yaw_rate_scale;          // Raw value × scale = rad/s
};
```

**Column mapping for each format:**

| Field | Old Format | CAN Format |
|---|---|---|
| Latitude | `gps_lat` (degrees) | `0x400_Latitude` (×10⁻⁷ → degrees) |
| Longitude | `gps_lon` (degrees) | `0x400_Logitude` (×10⁻⁷ → degrees) |
| Speed | `gps_speed` (m/s) | `0x408_Velocity` (×10⁻³ → m/s) |
| Yaw angle | `imu_yaw` (degrees) | `0x388_Y` (degrees) — if empty, computed from quaternion |
| Yaw rate | `imu_yaw_rate` (rad/s) | `0x288_g_z` (×π/180 → rad/s) |

**GPS presence detection:**  
Changed from checking if `string_row[0]` is empty (which was the latitude column only in the old format) to checking if the actual latitude column (identified by `ColumnMap`) is non-empty and non-NaN.

**Yaw from quaternion:**  
When `0x388_Y` is empty (as in the CAN dataset), yaw is computed from the quaternion columns (`0x488_q_w/x/y/z`) using the standard ZYX Euler angle extraction:

```cpp
double yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
```

**First GPS fix handling:**  
Added logic to find the first row with valid GPS data for EKF initialization, since the CAN dataset has ~36 rows of IMU-only data before the first GPS fix.

---

### 2. `ekf.cpp` — Fix Division by Zero in Propagation

**What changed:**  
Added a straight-line motion fallback when yaw rate (omega) is near zero.

**Before:**
```cpp
double r = v / omega;  // BOOM when omega = 0
double dx = -r * sin(theta) + r * sin(theta + dtheta);
double dy =  r * cos(theta) - r * cos(theta + dtheta);
```

**After:**
```cpp
if (std::abs(omega) < 1e-6) {
    // Straight-line motion: avoid division by zero
    dx = v * cos(theta) * dt;
    dy = v * sin(theta) * dt;
    G << 1, 0, -v*sin(theta)*dt,
         0, 1,  v*cos(theta)*dt,
         0, 0, 1;
} else {
    // Circular arc motion (original formula)
    double r = v / omega;
    dx = -r*sin(theta) + r*sin(theta + dtheta);
    dy =  r*cos(theta) - r*cos(theta + dtheta);
    G << 1, 0, -r*cos(theta) + r*cos(theta+dtheta),
         0, 1, -r*sin(theta) + r*sin(theta+dtheta),
         0, 0, 1;
}
```

**Why this is correct:**  
The circular arc formula `dx = (v/ω)[-sin(θ) + sin(θ+ωdt)]` has the limit as ω→0 of `v·cos(θ)·dt` (by L'Hôpital's rule or Taylor expansion). The Jacobian G follows the same limit.

---

### 3. `visualize.py` — Rewritten to Match `visualise.ipynb`

**What changed:**  
Completely rewrote `visualize.py` to mirror all 6 plots from the original Jupyter notebook (`visualise.ipynb`), with input-based output naming and pre-GPS row filtering.

**All 6 plots (matching notebook cells):**

| # | Plot | Notebook Cell | Output File |
|---|---|---|---|
| 1 | Full trajectory: State vs GPS | Cell 2 | `{base}.png` |
| 2 | Yaw arrows on full trajectory (every 100 steps) | Cell 3 | `{base}_ekf_results.png` |
| 3 | Zoomed scatter (200-step window) | Cell 4 | `{base}_output_final.png` |
| 4 | Zoomed yaw arrows (200-step window) | Cell 5 | `{base}_zoomed_yaw_arrows.png` |
| 5 | Yaw angle time series | Cell 7 | `{base}_yaw_timeseries.png` |
| 6 | Quiver plot: GPS path with yaw arrows (downsampled) | Cell 8 | `{base}_quiver.png` |

Where `{base}` is derived from the input filename (e.g., `can_log_vcu_20260322_150521_decoded`).

**Input-based output naming:**  
```bash
python3 visualize.py datasets/can_log_vcu_20260322_150521_decoded.csv
# Generates:
#   can_log_vcu_20260322_150521_decoded.png
#   can_log_vcu_20260322_150521_decoded_ekf_results.png
#   can_log_vcu_20260322_150521_decoded_output_final.png
#   can_log_vcu_20260322_150521_decoded_zoomed_yaw_arrows.png
#   can_log_vcu_20260322_150521_decoded_yaw_timeseries.png
#   can_log_vcu_20260322_150521_decoded_quiver.png
```

**Pre-GPS row filtering:**  
Before the first GPS fix (~36 rows in the CAN dataset), the GPS coordinates are (0°, 0°) which converts to a bogus UTM position far from the actual trajectory. These rows are now automatically detected and filtered out using a median-based distance check, preventing them from distorting the plot scale.

**Relative coordinates:**  
All plots now use relative coordinates (subtracting the first valid GPS position as origin), so the trajectory is centered near (0, 0) — matching the style of the original notebook plots.

---

### 4. `run.sh` — Updated Visualization Command

**What changed:**  
The `run.sh` script now shows the correct `visualize.py` command with the input filename, so the output PNGs are named after the dataset:

```bash
echo "  python3 visualize.py ${INPUT_CSV}"
```

---

### 5. `.gitignore` — Added `build/` directory

```
build/
```

---

## Testing

| Dataset | Format Detected | Rows | NaN in output | Result |
|---|---|---|---|---|
| `localization_log2.csv` | OLD | 58,174 | 0 | ✅ Matches original behavior |
| `datasets/can_log_vcu_20260322_150521_decoded.csv` | CAN | 30,723 | 0 | ✅ EKF tracks GPS within ~1-2m |

---

## How to Run

```bash
# Build + run EKF with new CAN dataset
./run.sh datasets/can_log_vcu_20260322_150521_decoded.csv

# Build + run EKF with old dataset (still works)
./run.sh localization_log2.csv

# Visualize (output PNGs named after input)
python3 visualize.py datasets/can_log_vcu_20260322_150521_decoded.csv

# Visualize with default naming
python3 visualize.py
```

## Generated Output Files

For input `datasets/can_log_vcu_20260322_150521_decoded.csv`:

| File | Description |
|---|---|
| `build/output_utm.csv` | EKF output (easting, northing, yaw, state_x, state_y, state_yaw) |
| `can_log_vcu_20260322_150521_decoded.png` | Full trajectory plot |
| `can_log_vcu_20260322_150521_decoded_ekf_results.png` | Yaw arrows on trajectory |
| `can_log_vcu_20260322_150521_decoded_output_final.png` | Zoomed scatter view |
| `can_log_vcu_20260322_150521_decoded_zoomed_yaw_arrows.png` | Zoomed yaw arrows |
| `can_log_vcu_20260322_150521_decoded_yaw_timeseries.png` | Yaw angle time series |
| `can_log_vcu_20260322_150521_decoded_quiver.png` | Quiver plot with yaw directions |
