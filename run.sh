#!/bin/bash
# Build and run the Extended Kalman Filter GPS/IMU fusion
set -e

INPUT_CSV="${1:-localization_log2.csv}"
PLOT_OUTPUT_DIR="${2:-}"

echo "=== Building Extended Kalman Filter GPS/IMU ==="
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
if command -v nproc >/dev/null 2>&1; then
  JOBS="$(nproc)"
elif command -v getconf >/dev/null 2>&1; then
  JOBS="$(getconf _NPROCESSORS_ONLN)"
else
  JOBS=1
fi
make -j"${JOBS}"

echo ""
echo "=== Running EKF ==="
./hav_cpp_from_python_2 "../${INPUT_CSV}"

echo ""
echo "=== Done! ==="
echo "Output saved to: build/output_utm.csv"
echo "Input used: ${INPUT_CSV}"
echo ""
if [ -n "${PLOT_OUTPUT_DIR}" ]; then
  echo "=== Generating plots ==="
  cd ..
  python3 visualize.py "${INPUT_CSV}" "${PLOT_OUTPUT_DIR}"
  echo ""
  echo "Plots saved to: ${PLOT_OUTPUT_DIR}"
else
  echo "To visualize results, run:"
  echo "  python3 visualize.py ${INPUT_CSV}"
fi
