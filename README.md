# Paper
D. Ellinoudis, L. Doitsidis and A. Amanatiadis, "Robust Multisensor Localization Using Sparse Landmarks for Autonomous Driving," 2024 32nd Mediterranean Conference on Control and Automation (MED), Chania - Crete, Greece, 2024, pp. 482-487, doi: 10.1109/MED61351.2024.10566202.
keywords: {Location awareness;Global navigation satellite system;Laser radar;System performance;Vehicle safety;Sensor systems;Sensors}, 

# Sensor Fusion of GPS and IMU with Extended Kalman Filter for Localization in Autonomous Driving
# Algorithm
![alt text](https://github.com/Janudis/EKF_GPS_IMU/blob/master/Extended-Kalman-Filter-Step.png)

This code implements an Extended Kalman Filter (EKF) for fusing Global Positioning System (GPS) and Inertial Measurement Unit (IMU) measurements. The goal is to estimate the state (position and orientation) of a vehicle using both GPS and IMU data.

In our case, IMU provide data more frequently than GPS. Here is a step-by-step description of the process:
1) Initialization: Firstly, initialize your EKF state [position, velocity, orientation] using the first GPS and IMU reading. The covariance matrix (P) should also be initialized to reflect initial uncertainty.
2) Prediction step (also known as Time Update): In the prediction step, you make a prediction of the current state using the process model and the previously estimated state. In our case, the state can be represented as [position, velocity, orientation]. We also need to predict the state covariance matrix (P) at this point.
3) Update step (also known as Measurement Update): In the update step, we correct the predicted state using the measurement data. GPS frequency is 4 Hz.
4) Estimate update: With the Kalman gain, we can compute the updated (a posteriori) state estimate by combining the predicted state estimate and the weighted difference between the actual measurement and the measurement predicted by the a priori estimate (also known as the measurement residual).  
5) Covariance update: We also compute the updated (a posteriori) estimate covariance (P). The a posteriori state and covariance estimates at the current time become the a priori estimates for the next time step.
6) Repeat steps 2-5: This process is then repeated for each time step, using the a posteriori estimates from the previous time step as the a priori estimates for the current step.  

# Dependencies
1) C++ compiler supporting C++11 or higher
  
2) Eigen library (for linear algebra operations)

# Usage
1) Install the required dependencies and ensure they are properly linked in your build environment.
2) Place the input data file (localization_log2.csv) in the same directory as the code files.
3) Compile the code using a C++ compiler.
4) Change the CMakeLists.txt before running the compiled executable.
5) The estimated position and orientation will be saved in the output_utm.csv file.

# Code Structure

| File | Description |
|------|-------------|
| `main_utm_2.cpp` | Main entry point — reads CSV input, runs EKF, writes output CSV |
| `ekf.h` / `ekf.cpp` | Extended Kalman Filter class (initialize, predict, update) |
| `geo_ned.h` / `geo_ned.cpp` | Helper functions for geodetic (WGS84) ↔ ENU coordinate conversion |
| `utm.h` / `utm.cpp` | UTM coordinate conversion (Transverse Mercator projection) |
| `run.sh` | Build & run script (one command) |
| `visualize.py` | Python visualization script for results |
| `visualise.ipynb` | Jupyter notebook for interactive visualization |
| `localization_log2.csv` | Sample input data (GPS/IMU sensor log) |
| `lib/Eigen/` | Eigen linear algebra library (header-only) |

# Input Data Format
The input data is expected to be in a CSV file (localization_log2.csv) with the following columns:

Timestamp (in nanoseconds)

Latitude (in degrees, WGS84)

Longitude (in degrees, WGS84)

Altitude (in meters, WGS84)

Forward velocity (in meters per second)

Yaw rate (in radians per second)

# Output
The output of the code is a CSV file (output_utm.csv) containing the estimated position and orientation in UTM coordinates. The file has the following columns:

Easting (UTM coordinate, in meters)

Northing (UTM coordinate, in meters)

Yaw (orientation angle, in radians)

Estimated X position (in meters)

Estimated Y position (in meters)

Estimated yaw (in radians)

# Adjusting Parameters
The code provides options for adjusting the standard deviation of the observation noise for x and y coordinates (xy_obs_noise_std), the yaw rate (yaw_rate_noise_std), and the forward velocity (forward_velocity_noise_std). These parameters can be modified in the code to suit your specific scenario and sensor characteristics.

# Results
![alt text](https://github.com/Janudis/EKF_GPS_IMU/blob/master/output_final.png)


