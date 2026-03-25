#!/bin/bash
# Build and run the Extended Kalman Filter GPS/IMU fusion
set -e

echo "=== Building Extended Kalman Filter GPS/IMU ==="
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)

echo ""
echo "=== Running EKF ==="
cp ../localization_log2.csv . 2>/dev/null || true
./hav_cpp_from_python_2

echo ""
echo "=== Done! ==="
echo "Output saved to: build/output_utm.csv"
echo ""
echo "To visualize results, run:"
echo "  python3 visualize.py"
