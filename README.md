# Paper
D. Ellinoudis, L. Doitsidis and A. Amanatiadis, "Robust Multisensor Localization Using Sparse Landmarks for Autonomous Driving," 2024 32nd Mediterranean Conference on Control and Automation (MED), Chania - Crete, Greece, 2024, pp. 482-487, doi: 10.1109/MED61351.2024.10566202.
keywords: {Location awareness;Global navigation satellite system;Laser radar;System performance;Vehicle safety;Sensor systems;Sensors}, 

# Sensor Fusion of GPS and IMU with Extended Kalman Filter for Localization in Autonomous Driving

## Algorithm
![alt text](https://github.com/Janudis/EKF_GPS_IMU/blob/master/Extended-Kalman-Filter-Step.png)

This code implements an Extended Kalman Filter (EKF) for fusing Global Positioning System (GPS) and Inertial Measurement Unit (IMU) measurements. The goal is to estimate the state (position and orientation) of a vehicle using both GPS and IMU data.

**EKF Steps:**
1. **Initialization** — Initialize state [x, y, yaw] from first GPS position and IMU quaternion yaw.
2. **Prediction (propagate)** — Predict state using bicycle/circular-arc motion model with wheel speed and gyro yaw rate.
3. **GPS Update** — Correct predicted (x, y) position using GPS observation (when available).
4. **Yaw Update** — Correct predicted yaw using IMU quaternion yaw observation.
5. **Repeat** — Steps 2–4 for each timestep.

## Dependencies

**Build:**
- C++ compiler supporting C++14 or higher (GCC, Clang, MSVC)
- CMake 3.22+
- Eigen library (included in `lib/` — no separate installation needed)

**Visualization:**
- Python 3
- matplotlib, numpy, pandas

```bash
pip3 install matplotlib numpy pandas
```

---

## Quick Start (One Command)

```bash
# Build and run with the default dataset (localization_log2.csv)
./run.sh

# Build and run with a CAN-decoded dataset
./run.sh datasets/can_log_vcu_20260322_150521_decoded.csv

# Then visualize the results (generates 6 PNG plots)
python3 visualize.py datasets/can_log_vcu_20260322_150521_decoded.csv
```

---

## Step-by-Step Usage

### Step 1: Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
cd ..
```

### Step 2: Run the EKF on a Dataset

```bash
# Option A: Original dataset
cd build && ./hav_cpp_from_python_2 ../localization_log2.csv && cd ..

# Option B: CAN-decoded dataset
cd build && ./hav_cpp_from_python_2 ../datasets/can_log_vcu_20260322_150521_decoded.csv && cd ..
```

This produces `build/output_utm.csv` containing the EKF results.

### Step 3: Generate Visualization Plots

```bash
# Pass the same input CSV filename to get named output PNGs
python3 visualize.py datasets/can_log_vcu_20260322_150521_decoded.csv
```

This generates **6 PNG files** in the project root directory:

| Output File | Description |
|---|---|
| `can_log_vcu_20260322_150521_decoded.png` | Full trajectory: EKF state vs GPS |
| `can_log_vcu_20260322_150521_decoded_ekf_results.png` | Yaw arrows on full trajectory (every 100 steps) |
| `can_log_vcu_20260322_150521_decoded_output_final.png` | Zoomed scatter view (200-step window) |
| `can_log_vcu_20260322_150521_decoded_zoomed_yaw_arrows.png` | Zoomed yaw arrows |
| `can_log_vcu_20260322_150521_decoded_yaw_timeseries.png` | Yaw angle time series (ground truth vs EKF) |
| `can_log_vcu_20260322_150521_decoded_quiver.png` | Quiver plot: GPS path with yaw direction arrows |

If you run `python3 visualize.py` without arguments, the output files are named `ekf.png`, `ekf_ekf_results.png`, etc.

**Alternative: Jupyter Notebook**
```bash
jupyter notebook visualise.ipynb
```

---

## Supported Input Formats

The EKF **auto-detects** the CSV format from the header row:

### Format 1: Original (`localization_log2.csv`)

| Column | Field | Unit |
|---|---|---|
| `gps_lat` | Latitude | degrees |
| `gps_lon` | Longitude | degrees |
| `gps_speed` | Forward velocity | m/s |
| `imu_yaw` | Yaw angle | degrees |
| `imu_yaw_rate` | Yaw rate | rad/s |

### Format 2: CAN-Decoded (`datasets/can_log_vcu_*.csv`)

| Column | Field | Unit | Conversion |
|---|---|---|---|
| `0x400_Latitude` | Latitude | integer ×10⁷ | ×10⁻⁷ → degrees |
| `0x400_Logitude` | Longitude | integer ×10⁷ | ×10⁻⁷ → degrees |
| `0x408_Velocity` | Forward velocity | mm/s | ×10⁻³ → m/s |
| `0x388_Y` | Yaw angle | degrees | (empty → fallback to quaternion) |
| `0x288_g_z` | Gyro Z (yaw rate) | deg/s | ×π/180 → rad/s |
| `0x488_q_w/x/y/z` | Quaternion | unitless | → yaw via `atan2(...)` |

---

## Code Structure

| File | Description |
|------|-------------|
| `main_utm_2.cpp` | Main entry point — reads CSV, auto-detects format, runs EKF, writes output |
| `ekf.h` / `ekf.cpp` | Extended Kalman Filter class (initialize, propagate, update, update_yaw) |
| `geo_ned.h` / `geo_ned.cpp` | Geodetic (WGS84) ↔ ENU coordinate conversion helpers |
| `utm.h` / `utm.cpp` | UTM coordinate conversion (Transverse Mercator projection) |
| `run.sh` | One-command build & run script |
| `visualize.py` | Python visualization script — generates 6 plots |
| `visualise.ipynb` | Jupyter notebook for interactive visualization |
| `localization_log2.csv` | Sample input data (original GPS/IMU sensor log) |
| `datasets/` | CAN-decoded datasets |
| `lib/Eigen/` | Eigen linear algebra library (header-only, bundled) |
| `WRITEUP.md` | Detailed technical writeup of all changes and known issues |

## Output Format

The output file `build/output_utm.csv` contains:

| Column | Description |
|---|---|
| `easting` | GPS easting in UTM (meters) |
| `northing` | GPS northing in UTM (meters) |
| `yaw` | Ground truth yaw from IMU quaternion (radians) |
| `state_x` | EKF estimated easting (meters) |
| `state_y` | EKF estimated northing (meters) |
| `state_yaw` | EKF estimated yaw (radians) |

## Adjusting Parameters

EKF noise parameters are auto-tuned per format but can be modified in `main_utm_2.cpp`:

| Parameter | Description | Old Format | CAN Format |
|---|---|---|---|
| `xy_obs_noise_std` | GPS position noise (meters) | 5.0 | 15.0 |
| `yaw_rate_noise_std` | Gyro yaw rate noise (rad/s) | 0.02 | 0.01 |
| `forward_velocity_noise_std` | Wheel speed noise (m/s) | 0.3 | 0.1 |

Increase `xy_obs_noise_std` to trust GPS less (smoother trajectory, more drift).  
Decrease process noise values to trust IMU/wheel speed more.

## Results
![alt text](https://github.com/Janudis/EKF_GPS_IMU/blob/master/output_final.png)
