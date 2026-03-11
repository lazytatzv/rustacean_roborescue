#!/usr/bin/env bash
# ============================================================
# NUC 本番セットアップスクリプト
# Ubuntu 22.04/24.04 + Nix (multi-user) を前提
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "============================================"
echo " Rustacean RoboRescue — NUC Production Setup"
echo "============================================"

# ── 1. Nix インストール確認 ──
if ! command -v nix &>/dev/null; then
    echo "⚠️  Nix がインストールされていません。インストールします..."
    curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
    echo "✅ Nix インストール完了。シェルを再起動してから再実行してください。"
    exit 0
fi

echo "✅ Nix: $(nix --version)"

# flakes 有効確認
if ! nix flake --help &>/dev/null 2>&1; then
    echo "⚠️  Nix flakes が無効です。有効化してください:"
    echo "   echo 'experimental-features = nix-command flakes' >> ~/.config/nix/nix.conf"
    exit 1
fi

# ── 2. サブモジュール取得 ──
echo ""
echo "── Git サブモジュール取得 ──"
cd "$REPO_DIR"
git submodule update --init --recursive
echo "✅ サブモジュール OK"

# ── 3. Nix 環境のプリビルド (初回のみ時間がかかる) ──
echo ""
echo "── Nix 開発環境のビルド (初回は 30-60 分かかります) ──"
nix develop --impure --command echo "✅ Nix 環境 OK"

# ── 4. ワークスペースビルド ──
echo ""
echo "── ワークスペースビルド ──"
cd "$REPO_DIR/main_ws"
nix develop "$REPO_DIR" --impure --command bash -c '
    set -e
    just forge
    echo "✅ ビルド完了"
'

# ── 5. udev ルール ──
echo ""
echo "── udev ルール設定 ──"
UDEV_SRC="$SCRIPT_DIR/99-robot.rules"
UDEV_DST="/etc/udev/rules.d/99-robot.rules"

if [ -f "$UDEV_SRC" ]; then
    if [ -f "$UDEV_DST" ] && diff -q "$UDEV_SRC" "$UDEV_DST" &>/dev/null; then
        echo "✅ udev ルール: 変更なし"
    else
        sudo cp "$UDEV_SRC" "$UDEV_DST"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        echo "✅ udev ルール: インストール + リロード済み"
    fi
else
    echo "⚠️  $UDEV_SRC が見つかりません。手動で設定してください。"
fi

# ── 6. systemd サービス ──
echo ""
echo "── systemd サービス設定 ──"
SERVICE_SRC="$SCRIPT_DIR/roborescue.service"
SERVICE_DST="/etc/systemd/system/roborescue.service"

if [ -f "$SERVICE_SRC" ]; then
    # テンプレート変数を実際のパスに置換
    CURRENT_USER=$(whoami)
    sed \
        -e "s|__REPO_DIR__|$REPO_DIR|g" \
        -e "s|__USER__|$CURRENT_USER|g" \
        "$SERVICE_SRC" | sudo tee "$SERVICE_DST" > /dev/null

    sudo systemctl daemon-reload
    echo "✅ systemd サービス: インストール済み"
    echo ""
    echo "   有効化: sudo systemctl enable roborescue"
    echo "   起動:   sudo systemctl start roborescue"
    echo "   ログ:   journalctl -u roborescue -f"
else
    echo "⚠️  $SERVICE_SRC が見つかりません"
fi

# ── 完了 ──
echo ""
echo "============================================"
echo " セットアップ完了!"
echo ""
echo " ■ 手動起動:"
echo "   cd $REPO_DIR/main_ws"
echo "   nix develop $REPO_DIR --impure"
echo "   source install/setup.bash"
echo "   ros2 launch bringup system.launch.py"
echo ""
echo " ■ 自動起動 (systemd):"
echo "   sudo systemctl enable --now roborescue"
echo ""
echo " ■ オペレータ側:"
echo "   Foxglove Studio → ws://<THIS_IP>:8765"
echo "   operator/ ディレクトリの README.md 参照"
echo "============================================"
