#!/bin/bash
# Build and run the Extended Kalman Filter GPS/IMU fusion
set -e

INPUT_CSV="${1:-localization_log2.csv}"

echo "=== Building Extended Kalman Filter GPS/IMU ==="
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)

echo ""
echo "=== Running EKF ==="
./hav_cpp_from_python_2 "../${INPUT_CSV}"

echo ""
echo "=== Done! ==="
echo "Output saved to: build/output_utm.csv"
echo "Input used: ${INPUT_CSV}"
echo ""
echo "To visualize results, run:"
echo "  python3 visualize.py ${INPUT_CSV}"
