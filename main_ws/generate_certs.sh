#!/bin/bash
# QUIC用のCA証明書とサーバー証明書を生成するスクリプト
#
# 使い方:
#   cd main_ws
#   ./generate_certs.sh [ROBOT_IPS...]
#
# 例:
#   ./generate_certs.sh                              # デフォルトIP使用
#   ./generate_certs.sh 192.168.1.100 10.0.0.1       # カスタムIP指定

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
QUIC_DIR="$SCRIPT_DIR/src/bringup/config/quic"

# quicディレクトリがなければ作成
mkdir -p "$QUIC_DIR"
cd "$QUIC_DIR"

# デフォルトのIP (Tailscale + ローカル)
DEFAULT_IPS="127.0.0.1,0.0.0.0"

# 引数からIPを追加
if [ $# -gt 0 ]; then
    for ip in "$@"; do
        DEFAULT_IPS="$DEFAULT_IPS,$ip"
    done
else
    # 引数がない場合、現在のIPを自動検出
    DETECTED_IPS=$(hostname -I 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$' | tr '\n' ',' | sed 's/,$//')
    if [ -n "$DETECTED_IPS" ]; then
        DEFAULT_IPS="$DEFAULT_IPS,$DETECTED_IPS"
    fi
fi

echo "=== QUIC証明書生成スクリプト ==="
echo "出力先: $QUIC_DIR"
echo "対象IP: $DEFAULT_IPS"
echo ""

# 既存ファイルのバックアップ
if [ -f ca.crt ] || [ -f server.crt ]; then
    BACKUP_DIR="backup_$(date +%Y%m%d_%H%M%S)"
    mkdir -p "$BACKUP_DIR"
    [ -f ca.crt ] && mv ca.crt "$BACKUP_DIR/"
    [ -f ca.key ] && mv ca.key "$BACKUP_DIR/"
    [ -f ca.srl ] && mv ca.srl "$BACKUP_DIR/"
    [ -f server.crt ] && mv server.crt "$BACKUP_DIR/"
    [ -f server.key ] && mv server.key "$BACKUP_DIR/"
    echo "既存の証明書を $BACKUP_DIR にバックアップしました"
fi

# CA証明書を生成
echo "CA証明書を生成中..."
openssl req -x509 -newkey rsa:2048 -keyout ca.key -out ca.crt -days 365 -nodes \
    -subj "/CN=zenoh-ca" \
    -addext "basicConstraints=critical,CA:TRUE" \
    2>/dev/null

# サーバー証明書のCSRを生成
echo "サーバー証明書を生成中..."
openssl req -newkey rsa:2048 -keyout server.key -out server.csr -nodes \
    -subj "/CN=zenoh-router" \
    2>/dev/null

# SANを構築
SAN_LIST=""
IFS=',' read -ra IP_ARRAY <<< "$DEFAULT_IPS"
for ip in "${IP_ARRAY[@]}"; do
    ip=$(echo "$ip" | tr -d ' ')
    if [ -n "$ip" ]; then
        if [ -n "$SAN_LIST" ]; then
            SAN_LIST="$SAN_LIST,IP:$ip"
        else
            SAN_LIST="IP:$ip"
        fi
    fi
done

# CAでサーバー証明書に署名
openssl x509 -req -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial \
    -out server.crt -days 365 \
    -extfile <(echo "subjectAltName=$SAN_LIST") \
    2>/dev/null

# CSRは不要なので削除
rm -f server.csr

# 権限設定
chmod 600 ca.key server.key
chmod 644 ca.crt server.crt

echo ""
echo "=== 生成完了 ==="
ls -la ca.crt ca.key server.crt server.key
echo ""
echo "オペレーター側に ca.crt をコピーしてください:"
echo "  scp $QUIC_DIR/ca.crt operator@OPERATOR_IP:~/rustacean_roborescue/operator_ws/quic/server.crt"
