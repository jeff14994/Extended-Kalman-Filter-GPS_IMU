#!/usr/bin/env python3
"""
Visualize Extended Kalman Filter GPS/IMU fusion results.
Reads output_utm.csv from the build directory and plots:
  1. GPS trajectory vs EKF estimated trajectory
  2. Yaw angle comparison (ground truth vs estimated)
  3. Zoomed-in view of a section

Usage:
  python3 visualize.py                          # output: ekf_results.png
  python3 visualize.py my_dataset.csv           # output: my_dataset.png
"""

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))

# Find the output CSV
csv_path = os.path.join(script_dir, "build", "output_utm.csv")
if not os.path.exists(csv_path):
    print(f"Error: {csv_path} not found!")
    print("Run './run.sh' first to build and generate the output.")
    exit(1)

# Derive output PNG name from optional input argument
if len(sys.argv) > 1:
    input_name = sys.argv[1]
    # Strip directory and extension
    base = os.path.splitext(os.path.basename(input_name))[0]
else:
    base = "ekf"

df = pd.read_csv(csv_path)
print(f"Loaded {len(df)} data points from {csv_path}")

state_x = df['state_x']
state_y = df['state_y']
gps_x = df['easting']
gps_y = df['northing']
yaw = df['yaw']
state_yaw = df['state_yaw']

# --- Plot 1: Full trajectory comparison ---
fig, axes = plt.subplots(1, 3, figsize=(20, 6))

ax1 = axes[0]
ax1.plot(gps_x, gps_y, label='GPS (observed)', alpha=0.7, linewidth=1)
ax1.plot(state_x, state_y, label='EKF (estimated)', alpha=0.7, linewidth=1)
ax1.set_xlabel('Easting (m)')
ax1.set_ylabel('Northing (m)')
ax1.set_title('Full Trajectory: GPS vs EKF')
ax1.legend()
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)

# --- Plot 2: Yaw angle comparison ---
ax2 = axes[1]
ax2.plot(yaw.values, lw=1.5, label='Ground Truth Yaw', alpha=0.7)
ax2.plot(state_yaw.values, lw=1, label='EKF Estimated Yaw', color='r', alpha=0.7)
ax2.set_xlabel('Time Step')
ax2.set_ylabel('Yaw Angle (rad)')
ax2.set_title('Yaw Angle: Ground Truth vs EKF')
ax2.legend()
ax2.grid(True, alpha=0.3)

# --- Plot 3: Zoomed-in trajectory section ---
ax3 = axes[2]
n = len(gps_x)
start = n // 2
end = min(start + 200, n)
ax3.scatter(gps_x[start:end], gps_y[start:end], label='GPS', s=10, alpha=0.7)
ax3.scatter(state_x[start:end], state_y[start:end], label='EKF', s=10, alpha=0.7)
ax3.set_xlabel('Easting (m)')
ax3.set_ylabel('Northing (m)')
ax3.set_title(f'Zoomed View (steps {start}-{end})')
ax3.legend()
ax3.set_aspect('equal')
ax3.grid(True, alpha=0.3)

plt.tight_layout()

# Save all three PNGs with input-based naming
main_path = os.path.join(script_dir, f"{base}.png")
ekf_results_path = os.path.join(script_dir, f"{base}_ekf_results.png")
output_final_path = os.path.join(script_dir, f"{base}_output_final.png")

plt.savefig(main_path, dpi=150)
plt.savefig(ekf_results_path, dpi=150)
plt.savefig(output_final_path, dpi=150)
print(f"Saved plot to {main_path}")
print(f"Saved plot to {ekf_results_path}")
print(f"Saved plot to {output_final_path}")

plt.show()
