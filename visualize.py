#!/usr/bin/env python3
"""
Visualize Extended Kalman Filter GPS/IMU fusion results.
Mirrors all plots from visualise.ipynb as a standalone script.

Reads output_utm.csv from the build directory and generates:
  1. Full trajectory: State vs GPS
  2. Yaw arrows on full trajectory (sampled every 100 steps)
  3. Zoomed scatter view (200-step window from middle)
  4. Zoomed yaw arrows (200-step window from middle)
  5. Yaw angle time series (ground truth vs estimated)
  6. Quiver plot: GPS path with yaw & state_yaw arrows (downsampled)

Usage:
  python3 visualize.py                                              # base name: ekf
  python3 visualize.py datasets/can_log_vcu_20260322_150521_decoded.csv  # base name from input
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import sys
from datetime import datetime

script_dir = os.path.dirname(os.path.abspath(__file__))

# Find the output CSV
csv_path = os.path.join(script_dir, "build", "output_utm.csv")
if not os.path.exists(csv_path):
    print(f"Error: {csv_path} not found!")
    print("Run './run.sh' first to build and generate the output.")
    exit(1)

# Derive base name from optional input argument, with timestamp suffix
timestamp_suffix = datetime.now().strftime("_%Y%m%d_%H%M%S")
if len(sys.argv) > 1:
    input_name = sys.argv[1]
    base = os.path.splitext(os.path.basename(input_name))[0] + timestamp_suffix
else:
    base = "ekf" + timestamp_suffix

df_raw = pd.read_csv(csv_path)
print(f"Loaded {len(df_raw)} data points from {csv_path}")

# Filter out pre-GPS rows: detect by checking if easting/northing jump
# (pre-GPS rows have bogus UTM from lat=0,lon=0)
# Use the state_x of the first valid row as reference
median_easting = df_raw['state_x'].median()
valid_mask = (df_raw['easting'] - median_easting).abs() < 10000
if valid_mask.sum() < len(df_raw):
    print(f"Filtered out {(~valid_mask).sum()} pre-GPS rows")
df = df_raw[valid_mask].reset_index(drop=True)

# Use relative coordinates (subtract origin for cleaner plots)
origin_x = df['easting'].iloc[0]
origin_y = df['northing'].iloc[0]

state_x = df['state_x'] - origin_x
state_y = df['state_y'] - origin_y
gps_x = df['easting'] - origin_x
gps_y = df['northing'] - origin_y
yaw = df['yaw']
state_yaw = df['state_yaw']

print(f"Using {len(df)} valid rows, origin=({origin_x:.1f}, {origin_y:.1f})")

# ============================================================
# Plot 1: Full trajectory — State vs GPS  (notebook cell 2)
# ============================================================
fig1, ax1 = plt.subplots(figsize=(10, 8))
ax1.plot(state_x, state_y, label='State (EKF)')
ax1.plot(gps_x, gps_y, label='GPS')
ax1.legend()
ax1.set_xlabel('Easting (m)')
ax1.set_ylabel('Northing (m)')
ax1.set_title('Full Trajectory: EKF State vs GPS')
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)

path1 = os.path.join(script_dir, f"{base}.png")
fig1.savefig(path1, dpi=150, bbox_inches='tight')
print(f"Saved: {path1}")

# ============================================================
# Plot 2: Yaw arrows on full trajectory  (notebook cell 3)
# ============================================================
fig2, ax2 = plt.subplots(figsize=(10, 8))
arrow_length = 5
for idx in range(0, len(gps_x), 100):
    ax2.arrow(gps_x.iloc[idx], gps_y.iloc[idx],
              np.cos(yaw.iloc[idx]) * arrow_length,
              np.sin(yaw.iloc[idx]) * arrow_length,
              linewidth=1, color="red")
    ax2.arrow(gps_x.iloc[idx], gps_y.iloc[idx],
              np.cos(state_yaw.iloc[idx]) * arrow_length,
              np.sin(state_yaw.iloc[idx]) * arrow_length,
              linewidth=1, color="blue")
ax2.set_xlabel('Easting (m)')
ax2.set_ylabel('Northing (m)')
ax2.set_title('Yaw Arrows (red=ground truth, blue=EKF) — every 100 steps')
ax2.set_aspect('equal')
ax2.grid(True, alpha=0.3)

path2 = os.path.join(script_dir, f"{base}_ekf_results.png")
fig2.savefig(path2, dpi=150, bbox_inches='tight')
print(f"Saved: {path2}")

# ============================================================
# Plot 3: Zoomed scatter  (notebook cell 4)
# ============================================================
n = len(gps_x)
zoom_start = n // 2
zoom_end = min(zoom_start + 200, n)

fig3, ax3 = plt.subplots(figsize=(10, 8))
ax3.scatter(state_x[zoom_start:zoom_end], state_y[zoom_start:zoom_end], label='State (EKF)', s=10)
ax3.scatter(gps_x[zoom_start:zoom_end], gps_y[zoom_start:zoom_end], label='GPS', s=10)
ax3.legend()
ax3.set_xlabel('Easting (m)')
ax3.set_ylabel('Northing (m)')
ax3.set_title(f'Zoomed Scatter (steps {zoom_start}–{zoom_end})')
ax3.set_aspect('equal')
ax3.grid(True, alpha=0.3)

path3 = os.path.join(script_dir, f"{base}_output_final.png")
fig3.savefig(path3, dpi=150, bbox_inches='tight')
print(f"Saved: {path3}")

# ============================================================
# Plot 4: Zoomed yaw arrows  (notebook cell 5)
# ============================================================
fig4, ax4 = plt.subplots(figsize=(10, 8))
arrow_length_zoom = 1
for idx in range(zoom_start, zoom_end):
    ax4.arrow(gps_x.iloc[idx], gps_y.iloc[idx],
              np.cos(yaw.iloc[idx]) * arrow_length_zoom,
              np.sin(yaw.iloc[idx]) * arrow_length_zoom,
              linewidth=1, color="red")
    ax4.arrow(gps_x.iloc[idx], gps_y.iloc[idx],
              np.cos(state_yaw.iloc[idx]) * arrow_length_zoom,
              np.sin(state_yaw.iloc[idx]) * arrow_length_zoom,
              linewidth=1, color="blue")
ax4.set_xlabel('Easting (m)')
ax4.set_ylabel('Northing (m)')
ax4.set_title(f'Zoomed Yaw Arrows (steps {zoom_start}–{zoom_end})')
ax4.set_aspect('equal')
ax4.grid(True, alpha=0.3)

path4 = os.path.join(script_dir, f"{base}_zoomed_yaw_arrows.png")
fig4.savefig(path4, dpi=150, bbox_inches='tight')
print(f"Saved: {path4}")

# ============================================================
# Plot 5: Yaw angle time series  (notebook cell 7)
# ============================================================
fig5, ax5 = plt.subplots(1, 1, figsize=(9, 6))
ax5.plot(yaw, lw=2, label='ground-truth')
ax5.plot(state_yaw, lw=1, label='estimated', color='r')
ax5.set_xlabel('time elapsed')
ax5.set_ylabel('yaw angle [rad/s]')
ax5.set_title('Yaw Angle: Ground Truth vs EKF Estimated')
ax5.legend()
ax5.grid(True, alpha=0.3)

path5 = os.path.join(script_dir, f"{base}_yaw_timeseries.png")
fig5.savefig(path5, dpi=150, bbox_inches='tight')
print(f"Saved: {path5}")

# ============================================================
# Plot 6: Quiver plot — GPS path with yaw arrows  (notebook cell 8)
# ============================================================
downsample_rate = 10
gps_x_ds = gps_x[::downsample_rate]
gps_y_ds = gps_y[::downsample_rate]
yaw_ds = yaw[::downsample_rate]
state_yaw_ds = state_yaw[::downsample_rate]

fig6, ax6 = plt.subplots(figsize=(10, 10))
ax6.plot(gps_x_ds, gps_y_ds, label='GPS', color='black')
ax6.quiver(gps_x_ds, gps_y_ds, np.cos(yaw_ds), np.sin(yaw_ds),
           color='b', scale=20, label='Yaw (ground truth)')
ax6.quiver(gps_x_ds, gps_y_ds, np.cos(state_yaw_ds), np.sin(state_yaw_ds),
           color='r', scale=20, label='State_Yaw (EKF)')
ax6.legend()
ax6.set_title('GPS with Yaw and State_Yaw (quiver)')
ax6.set_xlabel('Easting (m)')
ax6.set_ylabel('Northing (m)')
ax6.set_aspect('equal')
ax6.grid(True, alpha=0.3)

path6 = os.path.join(script_dir, f"{base}_quiver.png")
fig6.tight_layout()
fig6.savefig(path6, dpi=150, bbox_inches='tight')
print(f"Saved: {path6}")

print(f"\nAll plots saved with base name: {base}")
plt.show()
