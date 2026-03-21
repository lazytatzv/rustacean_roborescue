# Operator Station

Foxglove Studio と `joy_node` を使って遠隔からロボットを操作・モニタリングするための最小構成と手順をまとめます。

## 概要

- ロボット側は Zenoh（QUIC/UDP 7447）でデータを公開します。
- Operator 側は Zenoh に接続し、`foxglove_bridge` を動かして Foxglove Studio と接続します（推奨: Operator 側でブリッジ起動）。

## 前提

- ROS2 が利用可能であること（ワークスペース or system ROS）。
- Foxglove Studio をローカルにインストール済みであること（Electron アプリ推奨）。

## セットアップ手順

1. Foxglove Studio のインストール

	- https://foxglove.dev/download からダウンロード（Electron アプリ）。
	- または Ubuntu: `sudo snap install foxglove-studio`

2. コントローラ入力 (`joy_node`)

```bash
cd operator
# 必要なら operator/zenoh_ope.json5 の ROBOT_IP を編集
nix develop --impure   # 必要であれば
ros2 run joy joy_node
```

3. Operator の Zenoh 設定

- `operator/zenoh_ope.json5` を編集してロボットの IP（または Tailscale IP）を指定してください。デフォルトは `mode: "client"` になっています。

4. Foxglove Bridge の起動（Operator）

```bash
cd operator
./start_foxglove_bridge.sh
```

Foxglove Studio の接続先はデフォルトで `ws://localhost:8765` を想定しています。

## テストと確認

- joy トピック確認: `ros2 topic echo /joy --once`
- ノード一覧: `ros2 node list`
- Zenoh 経由のトピック確認: `ros2 topic list` でロボット側のトピックが見えるか確認

ネットワーク到達性（Operator → Robot の UDP 7447）を確認する簡易コマンド:

```bash
ROBOT_IP=10.42.0.1
nc -u -v -z $ROBOT_IP 7447 || echo "UDP 7447 unreachable"
```

リポジトリに用意したテストスクリプト:

- `operator/check_zenoh_network.sh` — UDP 到達性チェック
- `operator/test_zenoh_ros.sh` — ROS2 トピックで publish/echo の簡易テスト
- `operator/local_zenoh_test.md` — 同一マシン上での zenohd テスト手順

## ローカル（1台）での簡易 Zenoh テスト

ターミナル A:
```bash
zenohd --config /dev/null --listen quic/0.0.0.0:7447
```

ターミナル B:
```bash
zenoh-sub -e quic/127.0.0.1:7447 /hello
```

ターミナル C:
```bash
zenoh-pub -e quic/127.0.0.1:7447 /hello "hello_local"
```

または ROS2 の helper を使用:

```bash
cd operator
./test_zenoh_ros.sh echo /zenoh_test   # 受信側
./test_zenoh_ros.sh pub /zenoh_test hello_local  # 送信側
```

## 追加ファイル（リポジトリ内）

- `operator/start_foxglove_bridge.sh` — Foxglove Bridge 起動スクリプト
- `operator/foxglove-bridge.service` — systemd ユニットのテンプレート
- `operator/check_zenoh_network.sh` — UDP 到達性チェック
- `operator/test_zenoh_ros.sh` — ROS2 ベースの疎通テスト
- `operator/local_zenoh_test.md` — ローカルテスト手順

## 運用上の注意点

- Foxglove は Operator ローカルで動かすのが安全で簡単です（8765 を公開しない）。
- Docker 化する場合は QUIC/UDP の扱いに注意（Linux では `--network host` が確実）。
- 遠隔接続は Tailscale などの WireGuard ベース VPN を使うと NAT 越えが簡単になりますが、DERP リレー経由だと遅延が増える点に注意してください。

---

問題があれば、この README をさらに簡潔化します。どの部分をもっと短くしたいですか？
