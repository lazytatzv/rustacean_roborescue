# Development Quickstart

このファイルは、新しい開発者がローカルで素早く開発環境を再現するための最短手順を示します。

前提
- Linux（Ubuntu 22.04 推奨）
- Nix を使う場合は `nix` がインストールされていること（下記参照）
- Docker を使う場合は `docker` と `docker-compose` がインストールされていること

1) Nix 開発シェル（推奨）

```bash
# flake による dev shell
nix develop --accept-flake-config

# ワークスペースへ
cd main_ws

# 1回だけ（初回ビルド・パッケージ生成）
just forge

# ビルド / インストール
colcon build --merge-install

# 環境を読み込む
source install/setup.bash
```

メモ
- フレーク環境で問題がある場合は `nix-shell` の代替を README の `NIX.md` を参照してください。

2) Docker / docker-compose（代替手順）

```bash
# ルートディレクトリで
docker-compose build
docker-compose up -d

# コンテナ内に入る例
docker-compose exec app bash
```

3) Justfile の主要コマンド
- `just check` : フォーマット・lint・簡単なテスト
- `just fmt` : フォーマット自動修正
- `just test` : テスト実行
- `just forge` : ワークスペースビルドのラッパー（使用例: 初回セットアップ）

4) ローカルで CI の簡易再現

```bash
# 1) フォーマット・linters
just fmt
pre-commit run --all-files

# 2) Python quick tests
python -m pytest -q main_ws/test_all_functions.py -k "not integration"

# 3) Rust quick checks
cd main_ws
cargo fmt --all -- --check
cargo clippy --workspace -- -D warnings
```

5) トラブルシューティング
- Nix の問題は `docs/NIX.md` を参照してください。
- Docker の権限問題は `sudo usermod -aG docker $USER` を実行後、再ログインしてください。

必要に応じて、このクイックスタートに devcontainer や `.vscode` 設定を追加できます。希望があれば作成します。
