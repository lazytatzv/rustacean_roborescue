#!/bin/bash
# ロボットからCA証明書を取得するスクリプト
#
# 使い方:
#   ./fetch_cert.sh [USER@ROBOT_IP]
#
# 例:
#   ./fetch_cert.sh                        # operator_config.yaml の設定を使用
#   ./fetch_cert.sh 100.117.111.73         # IP指定（ユーザー名はconfigから）
#   ./fetch_cert.sh yano@100.117.111.73    # ユーザー@IP指定

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
QUIC_DIR="$SCRIPT_DIR/quic"
CONFIG_FILE="$SCRIPT_DIR/launch/operator_config.yaml"

# configから設定を読み込む
ROBOT_IP=""
ROBOT_USER=""
ROBOT_CERT_PATH="working/rustacean_roborescue/main_ws/src/bringup/config/quic/ca.crt"

if [ -f "$CONFIG_FILE" ]; then
    ROBOT_IP=$(grep -E '^robot_ip:' "$CONFIG_FILE" | sed 's/robot_ip:[[:space:]]*["'\'']\?\([^"'\'']*\)["'\'']\?/\1/' | tr -d ' ')
    ROBOT_USER=$(grep -E '^robot_user:' "$CONFIG_FILE" | sed 's/robot_user:[[:space:]]*["'\'']\?\([^"'\'']*\)["'\'']\?/\1/' | tr -d ' ')
    CERT_PATH_CFG=$(grep -E '^robot_cert_path:' "$CONFIG_FILE" | sed 's/robot_cert_path:[[:space:]]*["'\'']\?\([^"'\'']*\)["'\'']\?/\1/' | tr -d ' ')
    [ -n "$CERT_PATH_CFG" ] && ROBOT_CERT_PATH="$CERT_PATH_CFG"
fi

# 引数があれば上書き
if [ $# -gt 0 ]; then
    if [[ "$1" == *@* ]]; then
        ROBOT_USER="${1%@*}"
        ROBOT_IP="${1#*@}"
    else
        ROBOT_IP="$1"
    fi
fi

if [ -z "$ROBOT_IP" ]; then
    echo "エラー: robot_ip が指定されていません"
    echo "使い方: ./fetch_cert.sh [USER@ROBOT_IP]"
    echo "または operator_config.yaml に robot_ip を設定してください"
    exit 1
fi

# ユーザー指定があれば追加
if [ -n "$ROBOT_USER" ]; then
    SCP_TARGET="${ROBOT_USER}@${ROBOT_IP}"
else
    SCP_TARGET="${ROBOT_IP}"
fi

echo "=== 証明書取得スクリプト ==="
echo "接続先: $SCP_TARGET"
echo "証明書パス: ~/$ROBOT_CERT_PATH"
echo ""

# quicディレクトリ作成
mkdir -p "$QUIC_DIR"

# 証明書を取得
echo "CA証明書を取得中..."
scp "${SCP_TARGET}:~/${ROBOT_CERT_PATH}" "$QUIC_DIR/server.crt"

echo ""
echo "=== 完了 ==="
ls -la "$QUIC_DIR/server.crt"
