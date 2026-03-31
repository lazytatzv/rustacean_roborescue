set shell := ["bash", "-c"]


default: nix

nix:
  # nixglを使う場合impureが必要
  nix develop --impure --accept-flake-config

sync:
  # 新しい環境で作業する場合はsyncしてsubmoduleの内容物を取ってくる
  git submodule update --init --recursive

nix-status:
  systemctl status nix-daemon

dev:
  # Start a development shell for interactive work (recommended)
  @echo "Starting nix dev shell (interactive). Use Ctrl-D to exit."
  nix develop --accept-flake-config || true

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


cachix:
  cachix watch-exec roborescue-nix -- nix develop --command true
