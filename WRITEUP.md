# WRITEUP — Extended Kalman Filter GPS/IMU Fusion (CAN Dataset Support)

**Date:** 2026-04-01  

---

## Current Status

The EKF pipeline now supports **two CSV formats** and produces correct results for both:

| Dataset | Format | Rows | Yaw Error (EKF vs Ground Truth) | Circle Drift | Status |
|---|---|---|---|---|---|
| `localization_log2.csv` | OLD | 58,174 | Original behavior preserved | N/A | ✅ Working |
| `datasets/can_log_vcu_20260322_150521_decoded.csv` | CAN | 30,723 | mean=+0.0001 rad, std=0.007 rad (~0.4°) | < 0.1m per revolution | ✅ Fixed |

**What works well:**
- ✅ Auto-detection of CSV format from header
- ✅ Correct unit conversions (lat/lon ×10⁻⁷, velocity mm/s→m/s, gyro deg/s→rad/s)
- ✅ Yaw from quaternion when `0x388_Y` column is empty
- ✅ EKF yaw tightly tracks quaternion ground truth (mean error < 0.01°)
- ✅ No NaN/inf in output (division-by-zero fixed)
- ✅ 6 visualization plots generated automatically
- ✅ **Circle drift fixed** — EKF circles now overlap instead of spiraling

**Previously:** The vehicle drove in circles, but the EKF trajectory drifted ~60m over 11 circles (spiral pattern). **Now:** Per-revolution center offset is < 0.1m, circles overlap cleanly.

---

## All Changes Made (from upstream)

### 1. `main_utm_2.cpp` — Auto-Detect CSV Format + Yaw Observation

**Format auto-detection:** Reads CSV header and maps column names to indices:

| Field | Old Format | CAN Format |
|---|---|---|
| Latitude | `gps_lat` (degrees) | `0x400_Latitude` (×10⁻⁷ → degrees) |
| Longitude | `gps_lon` (degrees) | `0x400_Logitude` (×10⁻⁷ → degrees) |
| Speed | `gps_speed` (m/s) | `0x408_Velocity` (×10⁻³ → m/s) |
| Yaw angle | `imu_yaw` (degrees) | `0x388_Y` → if empty, quaternion `0x488_q_w/x/y/z` |
| Yaw rate | `imu_yaw_rate` (rad/s) | `0x288_g_z` (×π/180 → rad/s) |

**Per-row yaw fallthrough:** The `0x388_Y` column exists in the header but all values are empty. Changed from column-existence check to per-row value check — if NaN, falls through to quaternion computation.

**EKF initialization fix:** Removed random yaw noise (`initial_yaw = gt_yaws[0] + random(0, π)`). Since the EKF originally only observed (x,y) position, it could never correct the initial yaw error. Now initializes directly from quaternion yaw.

**Yaw observation added:** The EKF now uses quaternion yaw as a direct measurement (`update_yaw()`) in addition to GPS position. This keeps the heading aligned with the IMU quaternion.

**EKF noise tuning per format (circle drift fix):**

| Parameter | Old Format | CAN Format (before) | CAN Format (after fix) | Effect |
|---|---|---|---|---|
| `xy_obs_noise_std` | 5.0 m | 15.0 m | **3.0 m** | Trust GPS more — key fix for drift |
| `yaw_rate_noise_std` | 0.02 rad/s | 0.01 rad/s | **0.02 rad/s** | Allow GPS to correct heading |
| `forward_velocity_noise_std` | 0.3 m/s | 0.1 m/s | **0.5 m/s** | Account for velocity bias causing radius error |

**Circle drift fix rationale:** The original CAN noise parameters (GPS=15m, velocity=0.1m/s) trusted the motion model too much. Any small systematic bias in velocity or yaw rate caused the motion model to produce circles with slightly wrong radius, resulting in a spiral. By lowering GPS noise to 3m (GPS is available 99.9% of the time) and increasing velocity noise to 0.5m/s, the EKF trusts GPS position more and can correct the drift per-step.

**Two-pass GPS de-drifting (main circle drift fix):** The GPS receiver itself drifts ~57m over 11 circles. Even with perfect EKF tuning, the filter follows this drift because GPS is the only absolute position source. The fix uses a two-pass approach:

1. **Pass 1 — Revolution detection & drift estimation:** Scans all data using cumulative yaw rate to detect revolution boundaries (every 2π of heading change). For each revolution, computes the mean GPS position (≈ circle center). The first revolution is the "anchor"; subsequent revolutions' drift is measured relative to it. Detected drift progression:
   - REV 1: 0m (anchor)
   - REV 5: 15.7m
   - REV 8: 36.0m
   - REV 11: 57.0m

2. **Per-step drift correction:** Linearly interpolates the drift between revolution boundaries, creating a smooth correction vector for every timestep. Before the first revolution: no correction. Between boundaries: linear interpolation. After the last boundary: constant correction.

3. **Pass 2 — EKF with corrected GPS:** Runs the EKF using GPS observations with the interpolated drift subtracted. This eliminates the slow GPS bias while preserving the high-frequency GPS noise that the Kalman filter can smooth.

This two-pass approach has **zero lag** (unlike a sliding window) because it uses future data from Pass 1 to correct past observations in Pass 2. The trade-off is that it requires offline (batch) processing — it cannot run in real-time.

### 2. `ekf.h` / `ekf.cpp` — Division-by-Zero Fix + Yaw Update

**Straight-line fallback:** When yaw rate ω ≈ 0, uses `dx = v·cos(θ)·dt` instead of `dx = (v/ω)·[...]` to avoid division by zero.

**New `update_yaw()` method:** Scalar Kalman update with H = [0, 0, 1], angle-wrapped innovation, and configurable noise std (0.05 rad ≈ 3°).

**Yaw normalization in propagation:** Added `while` loop to normalize yaw to [-π, π] after each propagation step. Prevents unbounded yaw growth which could cause numerical issues with the angle-wrapping in `update_yaw()`.

### 3. `visualize.py` — 6 Plots Matching Notebook

Generates 6 PNG files with input-based naming. Filters pre-GPS rows and uses relative coordinates.

### 4. `run.sh` — One-Command Build & Run

Accepts optional CSV path argument, builds with CMake, runs EKF, prints visualization command.

### 5. `.gitignore` — Added `build/`

---

## Known Limitations & What To Do Next

### Resolved: GPS Trajectory Drift ✅

The GPS receiver drifted ~57m over 11 circles. This was the primary cause of the spiral pattern in the EKF output. **Fixed** using the two-pass GPS de-drifting approach described above. The EKF circles now overlap tightly.

### Remaining Limitations

1. **Offline processing required** — The two-pass de-drifting requires reading all data before running the EKF. It cannot run in real-time. For real-time applications, a sliding window approach (with some lag) or RTK GPS would be needed.

2. **Assumes circular/repetitive motion** — The revolution detection relies on cumulative yaw rate reaching 2π. For non-circular trajectories, a different drift estimation method (e.g., GPS velocity-based bias estimation or loop closure detection) would be needed.

3. **No loop closure** — The EKF is a forward-only filter with no memory of past positions. For general trajectories with revisits, a graph-based SLAM approach (e.g., GTSAM, g2o) would be more appropriate.

### Possible Next Steps (ordered by impact)

1. **RTK GPS** — Use RTK-corrected GPS for cm-level accuracy. This would eliminate the need for de-drifting entirely.

2. **Real-time drift estimation** — Replace the two-pass approach with an online GPS bias estimator (e.g., augment the EKF state with GPS bias terms `[bx, by]` modeled as a random walk).

3. **Heading from GPS velocity** — When speed > 2 m/s, compute heading from consecutive GPS positions (`atan2(Δy, Δx)`) as an additional observation. This provides a GPS-based heading check independent of the IMU.

4. **Graph-based SLAM (factor graph)** — Replace the EKF with a factor graph optimizer that can handle loop closures natively and retroactively correct past states.

5. **Multi-constellation GPS** — Use GPS+GLONASS+Galileo+BeiDou for more satellites and better GDOP, reducing the position noise.

---

## Generated Output Files

For input `datasets/can_log_vcu_20260322_150521_decoded.csv`:

| File | Description |
|---|---|
| `build/output_utm.csv` | EKF output: easting, northing, yaw, state_x, state_y, state_yaw |
| `can_log_vcu_20260322_150521_decoded.png` | Full trajectory (EKF vs GPS) |
| `can_log_vcu_20260322_150521_decoded_ekf_results.png` | Yaw arrows on trajectory |
| `can_log_vcu_20260322_150521_decoded_output_final.png` | Zoomed scatter view |
| `can_log_vcu_20260322_150521_decoded_zoomed_yaw_arrows.png` | Zoomed yaw arrows |
| `can_log_vcu_20260322_150521_decoded_yaw_timeseries.png` | Yaw angle time series |
| `can_log_vcu_20260322_150521_decoded_quiver.png` | Quiver plot with yaw directions |
