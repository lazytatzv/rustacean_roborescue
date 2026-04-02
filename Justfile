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


install-rust:
  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y


# udev rules deployment (roboclaw/stm32/dynamixel symlinks)
udev-install:
  sudo install -m 0644 deploy/99-robot.rules /etc/udev/rules.d/99-robot.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  @echo "udev rules installed and reloaded: /etc/udev/rules.d/99-robot.rules"

udev-status:
  ls -l /dev/roboclaw /dev/stm32 /dev/dynamixel_flipper /dev/dynamixel_arm 2>/dev/null || true