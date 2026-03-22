#!/usr/bin/env bash
set -e

echo "Entered Nix dev shell script for h264_republisher"

# Clean build for this package
rm -rf main_ws/build/main_ws/main_ws_build_h264 || true

which python || true
python -c 'import sys; print(sys.executable, sys.version)' || true

# Build only h264_republisher and capture full logs
colcon build --packages-select h264_republisher --merge-install --symlink-install --cmake-args -G Ninja 2>&1 | tee /tmp/colcon_h264_republisher.log
