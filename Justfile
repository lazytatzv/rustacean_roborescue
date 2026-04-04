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

# Install Nix cache settings for this repo to /etc/nix/nix.conf.d (requires sudo)
nix-cache-install:
  user="${SUDO_USER:-$USER}"; \
  user_home="$(getent passwd "$user" | cut -d: -f6)"; \
  user_nix_conf="$user_home/.config/nix/nix.conf"; \
  sudo install -d /etc/nix/nix.conf.d; \
  sudo touch /etc/nix/nix.conf; \
  if ! sudo grep -q '/etc/nix/nix.conf.d/roborescue-cachix.conf' /etc/nix/nix.conf; then \
    echo '!include /etc/nix/nix.conf.d/roborescue-cachix.conf' | sudo tee -a /etc/nix/nix.conf >/dev/null; \
  fi; \
  printf '%s\n' \
    'experimental-features = nix-command flakes' \
    'substituters = https://cache.nixos.org https://roborescue-nix.cachix.org https://nix-community.cachix.org https://ros.cachix.org' \
    'trusted-public-keys = cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY= roborescue-nix.cachix.org-1:qy3rP4VwHob/xePMW77gUxZVvPMz8izs86rIdruro0U= nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs= ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=' \
    'trusted-substituters = https://cache.nixos.org https://roborescue-nix.cachix.org https://nix-community.cachix.org https://ros.cachix.org' \
    "trusted-users = root ${user}" \
    'accept-flake-config = true' \
    | sudo tee /etc/nix/nix.conf.d/roborescue-cachix.conf >/dev/null; \
  sudo -u "$user" install -d "$user_home/.config/nix"; \
  sudo -u "$user" touch "$user_nix_conf"; \
  if ! sudo -u "$user" grep -q 'extra-substituters = .*roborescue-nix.cachix.org' "$user_nix_conf"; then \
    echo 'extra-substituters = https://roborescue-nix.cachix.org https://nix-community.cachix.org https://ros.cachix.org' | sudo -u "$user" tee -a "$user_nix_conf" >/dev/null; \
  fi; \
  if ! sudo -u "$user" grep -q 'extra-trusted-public-keys = .*roborescue-nix.cachix.org-1:' "$user_nix_conf"; then \
    echo 'extra-trusted-public-keys = roborescue-nix.cachix.org-1:qy3rP4VwHob/xePMW77gUxZVvPMz8izs86rIdruro0U= nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs= ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=' | sudo -u "$user" tee -a "$user_nix_conf" >/dev/null; \
  fi; \
  sudo systemctl restart nix-daemon; \
  echo "Installed system/user nix cache settings and restarted nix-daemon"

# Show effective Nix cache-related settings for troubleshooting
nix-cache-check:
  echo "== config files =="; \
  if [ -f /etc/nix/nix.conf ]; then \
    echo "-- /etc/nix/nix.conf"; \
    grep -nE 'include|substituters|trusted-public-keys|trusted-substituters|trusted-users|accept-flake-config' /etc/nix/nix.conf || true; \
  fi; \
  if [ -f /etc/nix/nix.conf.d/roborescue-cachix.conf ]; then \
    echo "-- /etc/nix/nix.conf.d/roborescue-cachix.conf"; \
    grep -nE 'substituters|trusted-public-keys|trusted-substituters|trusted-users|accept-flake-config' /etc/nix/nix.conf.d/roborescue-cachix.conf || true; \
  fi; \
  echo "== effective =="; \
  if nix config show >/dev/null 2>&1; then \
    nix config show; \
  elif nix show-config >/dev/null 2>&1; then \
    nix show-config; \
  else \
    echo "Could not read nix config with this nix CLI" >&2; \
    nix --help >&2; \
    exit 1; \
  fi | grep -E 'substituters|trusted-public-keys|trusted-substituters|trusted-users|accept-flake-config'; \
  if ! (nix config show 2>/dev/null || nix show-config 2>/dev/null) | grep -E '^substituters\s*=.*roborescue-nix.cachix.org' >/dev/null; then \
    echo "[WARN] roborescue-nix.cachix.org is NOT active in effective substituters"; \
    echo "       Run: just nix-cache-install"; \
  fi

# Enable compiler caches for local C/C++ and Rust builds
compiler-cache-setup:
  mkdir -p "${XDG_CACHE_HOME:-$HOME/.cache}/ccache" "${XDG_CACHE_HOME:-$HOME/.cache}/sccache" "$HOME/.config/sccache"
  if command -v ccache >/dev/null 2>&1; then \
    ccache --set-config=cache_dir="${XDG_CACHE_HOME:-$HOME/.cache}/ccache"; \
    ccache --set-config=max_size=20G; \
  fi
  if command -v sccache >/dev/null 2>&1; then \
    printf '%s\n' '[cache.disk]' "dir = \"${XDG_CACHE_HOME:-$HOME/.cache}/sccache\"" 'size = 21474836480' > "$HOME/.config/sccache/config"; \
    sccache --stop-server >/dev/null 2>&1 || true; \
    sccache --start-server >/dev/null 2>&1 || true; \
  fi
  echo "Configured ccache/sccache cache directories under ${XDG_CACHE_HOME:-$HOME/.cache}"

# Show compiler cache status and hit/miss counters
compiler-cache-status:
  echo "== env =="
  echo "CC=${CC:-<unset>}"
  echo "CXX=${CXX:-<unset>}"
  echo "RUSTC_WRAPPER=${RUSTC_WRAPPER:-<unset>}"
  echo "CCACHE_DIR=${CCACHE_DIR:-<unset>}"
  echo "SCCACHE_DIR=${SCCACHE_DIR:-<unset>}"
  echo
  if command -v ccache >/dev/null 2>&1; then \
    echo "== ccache =="; \
    ccache --show-config | rg 'cache_dir|max_size' || true; \
    ccache -s || true; \
  else \
    echo "ccache: not found"; \
  fi
  echo
  if command -v sccache >/dev/null 2>&1; then \
    echo "== sccache =="; \
    sccache --show-stats || true; \
  else \
    echo "sccache: not found"; \
  fi

# Reset compiler cache counters (keeps existing cache artifacts)
compiler-cache-reset-stats:
  if command -v ccache >/dev/null 2>&1; then ccache -z || true; fi
  if command -v sccache >/dev/null 2>&1; then sccache --zero-stats || true; fi
  echo "Reset ccache/sccache statistics counters"

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
  nix develop --accept-flake-config .#operator --command bash -lc 'cd operator_ws && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && export ZENOH_SESSION_CONFIG_URI=file://$PWD/zenoh_ope.json5 && export ZENOH_ROUTER_CHECK_ATTEMPTS=-1 && export ROS_DOMAIN_ID=0 && export ROS_LOCALHOST_ONLY=0 && ros2 daemon stop >/dev/null 2>&1 || true && ros2 launch launch/operator.launch.py use_foxglove:=false use_audio:=false use_joy:=true'

# Quick operator-side graph check under the same fixed env
operator-topic-list:
  nix develop --accept-flake-config .#operator --command bash -lc 'cd operator_ws && source install/setup.bash && export RMW_IMPLEMENTATION=rmw_zenoh_cpp && export ZENOH_SESSION_CONFIG_URI=file://$PWD/zenoh_ope.json5 && export ZENOH_ROUTER_CHECK_ATTEMPTS=-1 && export ROS_DOMAIN_ID=0 && export ROS_LOCALHOST_ONLY=0 && ros2 daemon stop >/dev/null 2>&1 || true && ros2 topic list'


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
  cachix watch-exec roborescue-nix -- nix develop --accept-flake-config --command true

# operator_ws 側の devShell も同じキャッシュへ投入
operator-cachix:
  cachix watch-exec roborescue-nix -- nix develop --accept-flake-config .#operator --command true

# devShell の全クロージャーを cachix へ明示 push する
# cachix watch-exec は "既に nix store にあるパス" を捕捉しないため、
# opencv など重いパッケージが store 済みのときはこちらを使う
nix-cache-push:
  echo "[nix-cache-push] collecting and pushing devShell closure ..."; \
  nix develop --accept-flake-config --command bash -c \
    'echo $buildInputs $nativeBuildInputs $propagatedBuildInputs $propagatedNativeBuildInputs' \
    | tr " " "\n" | grep "^/nix/store/" \
    | xargs -r nix-store --query --requisites \
    | sort -u \
    | cachix push roborescue-nix

# operator devShell 版
nix-cache-push-operator:
  echo "[nix-cache-push-operator] collecting and pushing operator devShell closure ..."; \
  nix develop --accept-flake-config .#operator --command bash -c \
    'echo $buildInputs $nativeBuildInputs $propagatedBuildInputs $propagatedNativeBuildInputs' \
    | tr " " "\n" | grep "^/nix/store/" \
    | xargs -r nix-store --query --requisites \
    | sort -u \
    | cachix push roborescue-nix


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
    env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  elif command -v nix >/dev/null 2>&1; then \
    echo "[dxl-scan] uv not found. Falling back to nix develop + uv."; \
    env -u PYTHONPATH -u VIRTUAL_ENV nix develop --accept-flake-config --command env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  fi

dxl-scan-arm *extra_args:
  if command -v uv >/dev/null 2>&1; then \
    env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  elif command -v nix >/dev/null 2>&1; then \
    echo "[dxl-scan] uv not found. Falling back to nix develop + uv."; \
    env -u PYTHONPATH -u VIRTUAL_ENV nix develop --accept-flake-config --command env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --baud 1000000 --both-protocols --allow-empty {{extra_args}}; \
  fi

dxl-scan-flipper-sweep:
  if command -v uv >/dev/null 2>&1; then \
    env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  elif command -v nix >/dev/null 2>&1; then \
    echo "[dxl-scan] uv not found. Falling back to nix develop + uv."; \
    env -u PYTHONPATH -u VIRTUAL_ENV nix develop --accept-flake-config --command env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  fi

dxl-scan-arm-sweep:
  if command -v uv >/dev/null 2>&1; then \
    env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  elif command -v nix >/dev/null 2>&1; then \
    echo "[dxl-scan] uv not found. Falling back to nix develop + uv."; \
    env -u PYTHONPATH -u VIRTUAL_ENV nix develop --accept-flake-config --command env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 2000000 3000000 4000000 --allow-empty; \
  fi

# Faster sweeps for bring-up: narrow ID range + common baud rates only
dxl-scan-flipper-fast:
  if command -v uv >/dev/null 2>&1; then \
    env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 --min-id 1 --max-id 32 --allow-empty; \
  elif command -v nix >/dev/null 2>&1; then \
    echo "[dxl-scan] uv not found. Falling back to nix develop + uv."; \
    env -u PYTHONPATH -u VIRTUAL_ENV nix develop --accept-flake-config --command env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 --min-id 1 --max-id 32 --allow-empty; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_flipper --both-protocols --bauds 1000000 57600 115200 --min-id 1 --max-id 32 --allow-empty; \
  fi

dxl-scan-arm-fast:
  if command -v uv >/dev/null 2>&1; then \
    env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 --min-id 1 --max-id 40 --allow-empty; \
  elif command -v nix >/dev/null 2>&1; then \
    echo "[dxl-scan] uv not found. Falling back to nix develop + uv."; \
    env -u PYTHONPATH -u VIRTUAL_ENV nix develop --accept-flake-config --command env -u PYTHONPATH -u VIRTUAL_ENV uv run --project tools/maintenance python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 --min-id 1 --max-id 40 --allow-empty; \
  else \
    echo "[dxl-scan] uv not found. Falling back to Python venv bootstrap."; \
    python3 -m venv tools/maintenance/.venv; \
    tools/maintenance/.venv/bin/python -m pip install -q -U pip; \
    tools/maintenance/.venv/bin/python -m pip install -q dynamixel-sdk pyserial; \
    tools/maintenance/.venv/bin/python tools/maintenance/dxl_scan.py --device /dev/dynamixel_arm --both-protocols --bauds 1000000 57600 115200 --min-id 1 --max-id 40 --allow-empty; \
  fi