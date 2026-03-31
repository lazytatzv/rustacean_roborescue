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
