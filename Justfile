set shell := ["bash", "-c"]

# Makefileでも良かったが、コメントなど書けるし便利なのでJustを使う
# cargoあたりでjustをインストールしておくことを推奨
# nix環境化ではflakeで入っているので気にする必要はない

default: nix

nix:
  # nixgldefaultを使う場合impureが必要だが、cachix使うときに都合悪いので使わない
  # nixglintelかnvidiaか
  # intel/amdならデフォルト設定しているnixglintelでいけるはず
  # accept~はcachix用
  nix develop --accept-flake-config

sync:
  # 新しい環境で作業する場合はsyncしてsubmoduleの内容物を取ってくる
  git submodule update --init --recursive

nix-status:
  systemctl status nix-daemon

dev:
  # Start a development shell for interactive work (recommended)
  @echo "Starting nix dev shell (interactive). Use Ctrl-D to exit."
  nix develop --accept-flake-config || true

# One command robot bringup (auto enters nix shell, sources overlay, launches system)
robot-up:
  nix develop --accept-flake-config --command bash -lc 'cd main_ws && source install/setup.bash && while true; do echo "[robot-up] launching system.launch.py"; ros2 launch bringup system.launch.py; code=$?; echo "[robot-up] launch exited code=$code; retry in 3s"; sleep 3; done'

# Minimal-mode robot bringup for communication-first recovery/debug
robot-up-min:
  nix develop --accept-flake-config --command bash -lc 'cd main_ws && source install/setup.bash && while true; do echo "[robot-up-min] launching minimal system"; ros2 launch bringup system.launch.py use_audio:=false use_lidar:=false use_camera:=false use_crawler:=false use_arm:=false use_flipper:=false use_imu:=false use_nav2:=false; code=$?; echo "[robot-up-min] launch exited code=$code; retry in 3s"; sleep 3; done'

# One command operator bringup with fixed Zenoh env (lightweight mode)
operator-up-min:
  nix develop --accept-flake-config .#operator --command bash -lc 'cd operator_ws && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && export RMW_ZENOH_CONFIG_URI=file://$PWD/zenoh_ope.json5 && export ZENOH_CONFIG_OVERRIDE="mode=\"peer\";connect/endpoints=[\"tcp/10.42.0.1:7447\"];scouting/multicast/enabled=false" && export ZENOH_ROUTER_CHECK_ATTEMPTS=-1 && export ROS_DOMAIN_ID=0 && export ROS_LOCALHOST_ONLY=0 && ros2 daemon stop >/dev/null 2>&1 || true && ros2 launch launch/operator.launch.py use_foxglove:=false use_audio:=false use_joy:=true'

# Quick operator-side graph check under the same fixed env
operator-topic-list:
  nix develop --accept-flake-config .#operator --command bash -lc 'cd operator_ws && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && export RMW_ZENOH_CONFIG_URI=file://$PWD/zenoh_ope.json5 && export ZENOH_CONFIG_OVERRIDE="mode=\"peer\";connect/endpoints=[\"tcp/10.42.0.1:7447\"];scouting/multicast/enabled=false" && export ZENOH_ROUTER_CHECK_ATTEMPTS=-1 && export ROS_DOMAIN_ID=0 && export ROS_LOCALHOST_ONLY=0 && ros2 daemon stop >/dev/null 2>&1 || true && ros2 topic list'


# TEST
# devcontainerは普段は使っていない
devcontainer:
  # Helper to show how to reopen in devcontainer
  @echo "To use the VS Code devcontainer: Open this folder in VS Code and select 'Remote-Containers: Reopen in Container'."

check-lint:
  @echo "Running linting and static checks"
  pre-commit run --all-files || true
  @echo "Rust: cargo fmt check and clippy"
  cd main_ws && cargo fmt --all -- --check || true
  cd main_ws && cargo clippy --workspace -- -D warnings || true
  @echo "Python: ruff"
  ruff check . || true


# cachixにバイナリキャッシュを上げる
# flake.nixを更新したら定期的にやっておく
cachix:
  cachix watch-exec roborescue-nix -- nix develop --command true

# operator_ws 側の devShell も同じキャッシュへ投入
operator-cachix:
  cachix watch-exec roborescue-nix -- nix develop --accept-flake-config .#operator --command true


install-rust:
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y


# udev rules deployment (roboclaw/stm32/dynamixel symlinks)
udev-install:
  sudo install -m 0644 deploy/99-robot.rules /etc/udev/rules.d/99-robot.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  @echo "udev rules installed and reloaded: /etc/udev/rules.d/99-robot.rules"

udev-status:
  @echo "== expected symlinks =="
  ls -l /dev/roboclaw /dev/stm32 /dev/dynamixel_flipper /dev/dynamixel_arm 2>/dev/null || true
  @echo ""
  @echo "== ttyUSB devices =="
  ls -l /dev/ttyUSB* 2>/dev/null || true
  @echo ""
  @echo "== by-id serial links =="
  ls -l /dev/serial/by-id 2>/dev/null || true

# Dynamixel ID scan utilities (uv preferred, venv fallback)
dxl-scan-flipper *extra_args:
  if command -v uv >/dev/null 2>&1; then \
    uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  fi

dxl-scan-arm *extra_args:
  if command -v uv >/dev/null 2>&1; then \
    uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  fi

dxl-scan-flipper-sweep:
  if command -v uv >/dev/null 2>&1; then \
    uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  fi

dxl-scan-arm-sweep:
  if command -v uv >/dev/null 2>&1; then \
    uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  fi