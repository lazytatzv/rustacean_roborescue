# 操作マニュアル

## 目次

1. [環境構築](#1-環境構築)
2. [ビルド](#2-ビルド)
3. [実機起動 (ロボット側)](#3-実機起動-ロボット側)
4. [オペレータ PC の起動](#4-オペレータ-pc-の起動)
5. [ネットワーク構成 (Zenoh)](#5-ネットワーク構成-zenoh)
6. [シミュレーション (Gazebo)](#6-シミュレーション-gazebo)
7. [コントローラ操作](#7-コントローラ操作)
8. [自律走行 (Nav2)](#8-自律走行-nav2)
9. [シリアルデバイス設定](#9-シリアルデバイス設定)
10. [トラブルシューティング](#10-トラブルシューティング)

---

## 1. 環境構築

### 1.1 前提条件

- **OS**: Linux (NixOS / Ubuntu 等)
- **Nix**: バージョン 2.18 以上 (flakes 有効)
- **GPU**: Gazebo GUI を使う場合は OpenGL 3.3 以上 (nixGL 経由)

### 1.2 初回セットアップ

以降のコマンドは、まず `nix develop --accept-flake-config` で開いた Nix dev shell の中で実行してください。

```bash
# リポジトリクローン (サブモジュール含む)
git clone --recursive <REPO_URL>
cd rustacean_roborescue

# サブモジュール取得
just sync
```

---

## 2. ビルド

### 2.1 フルビルド

```bash
# Nix dev shell へ入る
nix develop --accept-flake-config

cd main_ws
just forge
```

`just forge` は以下を順に実行します:
1. `rosidl_generator_rs` などの依存ジェネレータをビルド
2. `colcon build --merge-install` でワークスペース全体をビルド
3. `source install/setup.bash` (現在のシェルには手動で実行が必要)

### 2.2 差分ビルド

```bash
cd main_ws
colcon build --merge-install --packages-select <package_name>
```

---

## 3. 実機起動 (ロボット側)

### 3.1 Zenoh 設定の確認

`main_ws/src/bringup/config/zenoh_router.json5` が `mode: "router"` であることを確認します。
この設定は変更不要です（ロボット側は常に router として動作します）。

### 3.2 起動

```bash
cd main_ws
source install/setup.bash
ros2 launch bringup system.launch.py
```

**オプション引数**:

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `use_nav2` | `false` | `true` で Nav2 自律走行を同時に起動 |
| `use_audio` | `true` | `false` で音声転送を無効化 |

### 3.3 起動されるノード一覧

`system.launch.py` が起動するノード（サブ Launch を含む）:

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| `robot_state_publisher` | robot_state_publisher | URDF → TF |
| `zenoh_router_daemon` | zenohd | Zenoh ルーター (QUIC優先/TCPフォールバック, 7447/17447) |
| `spark_fast_lio` | spark-fast-lio | LiDAR-IMU オドメトリ |
| `slam_toolbox` | slam_toolbox | SLAM 地図生成 |
| `v4l2_camera` | v4l2_camera | USB カメラ (H.264 エンコード) |
| `qr_detector` | qr_detector_cpp | QR コード検出 |
| `crawler_driver` | crawler_driver | Roboclaw モータ制御 |
| `flipper_driver` | flipper_driver | Dynamixel フリッパ制御 |
| `arm_gripper_driver` | arm_driver | Dynamixel アーム + グリッパ |
| `sensor_gateway` | sensor_gateway | STM32 IMU ブリッジ |
| `joy_controller` | joy_controller | PS4 テレオペ処理 |
| `arm_controller` | arm_controller | アーム逆運動学 (IK) |

`launch_config.yaml` のデフォルトは部分接続向けです（`use_arm=false`, `use_flipper=false`, `use_imu=false`）。
機器を接続したら必要な項目だけ `true` に戻して起動してください。

### 3.4 systemd による自動起動（本番デプロイ）

```bash
# NUC に systemd ユニットをインストール
cd deploy
sudo cp roborescue.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable roborescue
sudo systemctl start roborescue

# ログ確認
journalctl -u roborescue -f
```

---

## 4. オペレータ PC の起動

### 4.1 ネットワーク設定（必須）

`operator_ws/zenoh_ope.json5` の `connect.endpoints` を、ロボット (NUC) の実際の IP アドレスに書き換えます:

```json5
{
  mode: "peer",
  connect: { endpoints: ["tcp/10.42.0.1:7447"] },  // ← Robot IP を変更
  scouting: { multicast: { enabled: false } },
  transport: { shared_memory: { enabled: true } }
}
```

`operator.launch.py` では `ZENOH_CONFIG_OVERRIDE` により同等の設定を強制しています。
これにより、`RMW_ZENOH_CONFIG_URI` が無視される環境でも localhost 誤接続を回避できます。

### 4.2 起動

```bash
cd operator_ws
ros2 launch launch/operator.launch.py
```

軽量起動で接続確認だけしたい場合は、リポジトリルートで以下を使えます:

```bash
just operator-up-min
```

### 4.3 Foxglove Studio の設定

1. Foxglove Studio を起動
2. 「Open connection」→「Foxglove WebSocket」→ `ws://127.0.0.1:8765`
3. カメラ映像: 「+」→「Video」パネル→ トピックに `/camera/image_raw/foxglove_video`
  （または `/camera_front/compressed_video`）を選択
4. LiDAR: 「+」→「3D」パネル → `/velodyne_points` または `/scan`

---

## 5. ネットワーク構成 (Zenoh)

### 5.1 アーキテクチャ概要

ロボットとオペレータ間の ROS 2 通信は `rmw_zenoh_cpp` (Zenoh) を使います。
ロボット側 router は **QUIC + TCP 待受** です。
オペレータ側は接続安定性を優先し、運用では **TCP 固定 (`tcp/<ROBOT_IP>:7447`)** を使います。

運用上の実態として、オペレータ側は `tcp/<ROBOT_IP>:7447` の単一接続を固定し、
`ZENOH_CONFIG_OVERRIDE` で確実に同設定を適用します。

ロボット側では `zenohd` が router として動作し、ROS 2 ノードは `zenoh_robot.json5` を使って client として接続します。オペレータ側は `zenoh_ope.json5` でロボット側 router に接続します。

```
Operator PC                                   Robot NUC
  RMW_ZENOH_CONFIG_URI=zenoh_ope.json5          zenohd (router daemon)
  mode: peer                                      zenoh_router.json5
  connect: tcp <ROBOT_IP>:7447 ───────────────►  listen: quic/tcp 0.0.0.0:7447
                                                       ▲
                                                  ROS2 ノード群
                                                    zenoh_robot.json5
                                                    mode: client
                                                    connect: quic/tcp 127.0.0.1:7447,17447
```

重要: **ロボット側の ROS 2 ノードは `zenoh_robot.json5` を使い、`zenohd` とは別プロセスとして動作します**。
ROS2 ノード自身が router になると `zenohd` とポート競合するため、ロボットノード用と router 用で設定ファイルを分けます。

### 5.2 ロボット側 Zenoh 設定

**ファイル**: `main_ws/src/bringup/config/zenoh_router.json5`

これは `zenohd` (router daemon) 用の設定です。ロボット側 ROS 2 ノードには `main_ws/src/bringup/config/zenoh_robot.json5` が使われます。

```json5
{
  mode: "router",
  listen: { endpoints: ["quic/0.0.0.0:7447", "tcp/0.0.0.0:7447"] },
  scouting: { multicast: { enabled: false } },
  transport: {
    shared_memory: { enabled: true },
    link: {
      tls: {
        listen_private_key: "quic/server.key",
        listen_certificate: "quic/server.crt"
      }
    }
  }
}
```

- `mode: "router"`: ロボット側は router として動作し、クライアントからの接続を受け付ける
- `quic/0.0.0.0:7447`, `tcp/0.0.0.0:7447`: QUIC 優先 + TCP フォールバックで待受
- `multicast: false`: Wi-Fi 環境でのブロードキャスト漏れを防止
- `shared_memory: true`: 同一マシン上のノード間は SHM (共有メモリ) で高速転送
- `network.launch.py` は QUIC 証明書 (`quic/server.crt`, `quic/server.key`) がない場合に自己署名証明書を自動生成します
- 7447 が他プロセスに占有されている場合は、ローカル zenohd を 17447 で起動して継続します

### 5.3 オペレータ側 Zenoh 設定

**ファイル**: `operator_ws/zenoh_ope.json5`

```json5
{
  mode: "peer",
  connect: { endpoints: ["tcp/10.42.0.1:7447"] },  // Robot IP
  scouting: { multicast: { enabled: false } },
  transport: { shared_memory: { enabled: true } }
}
```

- `mode: "peer"`: router 一時停止中でも operator ノード自体は起動を継続しやすい
- `connect.endpoints`: **ロボットの IP アドレスに変更必須**

### 5.4 環境変数の設定

`operator.launch.py` および `network.launch.py` が自動的に設定します:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export RMW_ZENOH_CONFIG_URI=file:///path/to/zenoh_xxx.json5
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE='mode="peer";connect/endpoints=["tcp/<ROBOT_IP>:7447"];scouting/multicast/enabled=false'
```

手動でターミナルから ROS 2 コマンドを実行する場合は、上記を事前に設定してください。

### 5.5 接続確認

```bash
# トピック一覧が表示されれば接続成功
just operator-topic-list

# /joy がリストに含まれるか確認（joy_nodeが動いていれば）
ros2 topic echo /joy --once
```

### 5.6 Wi-Fi 帯域の目安

| トピック | 帯域 |
|---------|------|
| `/camera/image_raw/ffmpeg` (H.264, 2Mbps) | ~2 Mbps |
| `/velodyne_points` (VLP-16, 15Hz) | ~8–15 Mbps |
| `/scan` (LaserScan) | < 0.1 Mbps |
| `/joy`, 制御指令 | < 0.1 Mbps |

> **Note**: LiDAR PointCloud2 は帯域を大きく消費します。Wi-Fi の帯域が限られている場合は
> オペレータ側の Foxglove で LiDAR トピックの購読を外すか、`/scan` のみを使ってください。

---

## 6. シミュレーション (Gazebo)

実機がなくても、Gazebo Harmonic を使用して自律走行やアームの動作をテスト可能です。

### 6.1 基本的な起動

```bash
# Nix dev shell へ入る
nix develop --accept-flake-config

cd main_ws
source install/setup.bash

# シミュレーション + SLAM (地図作成)
ros2 launch bringup simulation.launch.py use_slam:=true

# Gazebo GUI も開く場合
ros2 launch bringup simulation.launch.py use_slam:=true use_gz_gui:=true

# GPU なし / CI 向け
ros2 launch bringup simulation.launch.py headless:=true use_slam:=false use_rviz:=false
```

シミュレーションでは Gazebo が `/velodyne_points` を publish し、`pointcloud_to_laserscan` が `/scan` に変換します。SLAM Toolbox と Nav2 は原則として `/scan` を購読します。

### 6.2 自律走行 (Nav2) を有効にして起動

```bash
# シミュレーション + SLAM + Nav2
ros2 launch bringup simulation.launch.py use_slam:=true use_nav2:=true
```

---

## 7. コントローラ操作

PS4 コントローラで操作します。

### 7.1 モード一覧

| ボタン | モード | 説明 |
|--------|--------|------|
| **PS** | STOP | **緊急停止** (全出力遮断・ラッチ式) |
| **OPTIONS** | DRIVE | 走行・フリッパ操作 |
| **SHARE** | ARM | アーム操作 (IK) |

### 7.2 DRIVE モード（OPTIONS ボタンで切替）

| 入力 | 動作 |
|------|------|
| 左スティック Y | 左クローラ速度 (最大 0.7 m/s) |
| 右スティック Y | 右クローラ速度 (最大 0.7 m/s) |
| D-Pad 上/下 | 前後フリッパ速度 ±1000 |
| □/× ボタン | 左右フリッパ速度 ±1000 |
| L1/R1 | 左後/右後フリッパ速度 ±1000 |
| L2/R2 | 左前/右前フリッパ速度 ±1000 |

### 7.3 ARM モード（SHARE ボタンで切替）

| 入力 | 動作 |
|------|------|
| 左スティック X/Y | 手先 XY 並進 (0.3 m/s) |
| 右スティック Y | 手先 Z 方向 (0.3 m/s) |
| 右スティック X | Yaw 回転 (0.5 rad/s) |
| L2/R2 トリガー | Roll 回転 (0.5 rad/s) |
| D-Pad 上/下 | Pitch 回転 (0.5 rad/s) |

### 7.4 緊急停止 (E-Stop)

- **PS ボタン**で緊急停止。全モータが即座に停止します。
- **ラッチ式**: 一度発動するとノード再起動まで解除不可。
- E-Stop 解除は `ros2 launch bringup system.launch.py` の再起動が必要です。

---

## 8. 自律走行 (Nav2)

1. **環境の起動**: セクション 6.2 の手順でシミュレーション（または実機）を起動。
2. **地図生成**: ロボットをコントローラで少し走らせ、RViz または Foxglove 上で周囲の地図が描画されるのを待つ。
3. **目的地の指定**: RViz 上部の「Nav2 Goal」ボタンを押し、地図上の行きたい場所をクリック＆ドラッグ。
4. **自動走行**: Nav2 が経路を計算し、ロボットが自動で障害物を避けながら目的地まで走行します。

---

## 9. シリアルデバイス設定

### 9.1 udev ルールの適用

ロボット (NUC) の USB/UART デバイスを固定名で認識させるため、udev ルールを設定します。

```bash
sudo just udev-install
```

`just udev-install` は `deploy/99-robot.rules` を `/etc/udev/rules.d/99-robot.rules` に配置し、ルール再読み込みまで行います。

### 9.2 デバイス対応表

| デバイス名 | 接続先 | 用途 |
|-----------|--------|------|
| `/dev/roboclaw` | Roboclaw モータコントローラ (USB-UART) | クローラ左右モータ |
| `/dev/dynamixel_flipper` | Dynamixel U2D2 (フリッパ用) | フリッパ 4 軸 (ID 1–4) |
| `/dev/dynamixel_arm` | Dynamixel U2D2 (アーム用) | アーム 6 軸 (ID 21–26) + グリッパ (ID 10) |
| `/dev/stm32` | STM32 マイコン (USB-UART) | BNO055 IMU データ |

### 9.3 デバイスの確認

```bash
# デバイスが認識されているか確認
just udev-status
ls -la /dev/roboclaw /dev/dynamixel_flipper /dev/dynamixel_arm /dev/stm32

# シリアル通信テスト (IMU の例)
cat /dev/stm32
# → heading,roll,pitch,... の CSV が流れれば正常
```

### 9.4 権限設定

```bash
# ユーザーを dialout グループに追加（sudo 不要でシリアルデバイスにアクセス）
sudo usermod -aG dialout $USER
# 再ログイン後に有効になります
```

---

## 10. トラブルシューティング

### 10.1 Zenoh が接続しない

- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` が設定されているか確認。
- `operator_ws/zenoh_ope.json5` の IP アドレスが正しいか確認。
- ロボット側で `zenohd` が起動しているか確認: `ps aux | grep zenohd`
- ファイアウォールで TCP/7447 が開いているか確認。
- ロボット側で待受を確認: `ss -ltnp | grep 7447`
- `Connection refused` の場合はロボット側 `zenohd` 未起動または別プロセス競合。

### 10.2 `/joy` がロボットに届かない

- オペレータ側の `joy_node` が起動しているか確認: `ros2 topic echo /joy`
- Zenoh の接続状態を確認（10.1 参照）

### 10.3 カメラ映像が Foxglove に出ない

- ロボット側で `camera.launch.py` が起動しているか確認。
- `/camera/image_raw/foxglove_video` または `/camera_front/compressed_video` が存在するか確認:
  `ros2 topic list | grep compressed_video`
- Foxglove Studio の **Video** パネルで上記トピックが選択されているか確認（H.264 形式）。

### 10.4 Nav2 が経路を作らない

- 地図が十分にできていないか、目的地が障害物に近すぎる可能性があります。
- RViz 上でコストマップ（赤い影）を確認してください。
- `slam_toolbox` が active 状態か確認: `ros2 lifecycle get /slam_toolbox`

### 10.5 アームが動かない

- `arm_controller` が `/joint_states` を受信するまで指令を受け付けません（起動安全機能）。
- `arm_gripper_driver` が正常に起動しているか確認: `ros2 topic echo /joint_states`
- E-Stop が発動していないか確認: `ros2 topic echo /emergency_stop`

### 10.6 シリアルデバイスが見つからない

- udev ルールが適用されているか確認（セクション 9 参照）。
- USB ケーブルの接続を確認。
- `lsusb` でデバイスが認識されているか確認。

### 10.7 クローラ片側が止まらない / 暴走気味になる

- 最優先で機体を浮かせ、E-Stop と電源遮断を即実行できる状態で検証する。
- 直前ログに `ACK timeout` / `Write error: Input/output error` がある場合、コードより先に通信断を疑う。
- `crawler_driver` は通信断を検知すると fail-safe で切断扱いに入り、再接続を試行する。
- `main_ws/src/bringup/config/crawler_driver.yaml` で `debug_io: true` を有効化し、
  `cmd_vel`, qpps 指令値, 実速度, 通信断イベントを確認する。
- 配線入れ替えで症状が左右反転するかを確認し、物理系 (モータ/ギア) と上流 (配線/コントローラ/通信) を切り分ける。

### 10.8 Gazebo が起動しない / SEGV する

- `rescue_field.sdf` の `gz-sim-sensors-system` は `render_engine=ogre` を使います。GUI や OpenGL 周りが不安定な環境では `simulation.launch.py headless:=true` か `use_gz_gui:=false` で起動してください。
- それでも落ちる場合は、`nixGL` / Mesa の設定が合っているか、Gazebo が正しい GPU/ソフトウェアレンダリング経路で起動しているかを確認してください。
