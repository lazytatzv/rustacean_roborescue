set shell := ["bash", "-c"]


default: nix

nix:
  # nixglを使う場合impureが基本的に必要
  nix develop --impure

sync:
  # 新しい環境で作業する場合はsyncしてsubmoduleの内容物を取ってくる
  git submodule update --init --recursive

nix-status:
  systemctl status nix-daemon
