#!/usr/bin/env bash
set -e

echo "Running inside nix dev shell script"

# Verify python
echo "which python:" $(which python)
python - <<'PY'
import sys, numpy
print(sys.executable)
print(numpy.get_include())
PY

NUMPY_INC=$(python -c 'import numpy; print(numpy.get_include())')
PYTHON_EXECUTABLE=$(which python)

echo "PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}"
echo "NUMPY_INC=${NUMPY_INC}"

# Build only the package under test
colcon build --packages-select ros2_webrtc --merge-install --symlink-install \
  --cmake-args -DPython3_FIND_STRATEGY=LOCATION -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}" -DPython3_NumPy_INCLUDE_DIRS="${NUMPY_INC}" -G Ninja
