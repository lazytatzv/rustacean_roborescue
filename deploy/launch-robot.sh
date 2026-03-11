#!/usr/bin/env bash
# ============================================================
# NUC 本番起動スクリプト
# systemd から呼ばれる。手動実行も可。
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
MAIN_WS="$REPO_DIR/main_ws"

echo "[roborescue] Starting... ($(date))"
echo "[roborescue] Repo: $REPO_DIR"

cd "$MAIN_WS"

# nix develop 内で ROS 2 overlay を source して launch
exec nix develop "$REPO_DIR" --impure --command bash -c '
    source install/setup.bash
    echo "[roborescue] ROS 2 overlay loaded"
    echo "[roborescue] Launching system.launch.py ..."
    exec ros2 launch bringup system.launch.py
'
