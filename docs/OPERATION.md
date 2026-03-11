# 操作マニュアル

## 目次

1. [環境構築](#1-環境構築)
2. [ビルド](#2-ビルド)
3. [実機起動](#3-実機起動)
4. [シミュレーション](#4-シミュレーション)
5. [コントローラ操作](#5-コントローラ操作)
6. [自律走行 (Nav2)](#6-自律走行-nav2)
7. [シリアルデバイス設定](#7-シリアルデバイス設定)
8. [ネットワーク構成](#8-ネットワーク構成)
9. [本番デプロイ](#9-本番デプロイ)
10. [トラブルシューティング](#10-トラブルシューティング)

---

## 1. 環境構築

### 1.1 前提条件

- **OS**: Linux (NixOS / Ubuntu 等)
- **Nix**: バージョン 2.18 以上 (flakes 有効)
- **GPU**: Gazebo GUI を使う場合は OpenGL 3.3 以上 (nixGL 経由)

### 1.2 初回セットアップ

```bash
# リポジトリクローン (サブモジュール含む)
git clone --recursive <REPO_URL>
cd rustacean_roborescue

# サブモジュール取得 (クローン時に --recursive を忘れた場合)
just sync
# または: git submodule update --init --recursive
```

### 1.3 Nix 開発シェルに入る

```bash
# flake.nix で定義された全ツール (ROS 2 Jazzy, Rust nightly, Gazebo Harmonic 等) が使える
just nix
# または: nix develop --impure
```

> **`--impure` が必要な理由**: nixGL が `/run/opengl-driver` 等のホスト側 GPU ドライバを参照するため。

起動すると以下が表示される:

```
=======================================================
 Rustacean RoboRescue Dev Environment
 ROS: jazzy | Rust: nightly | Sim: Gazebo Harmonic
=======================================================
```

### 1.4 direnv (オプション)

`cd` するだけで自動で Nix シェルに入りたい場合:

```bash
nix profile install nixpkgs#direnv nixpkgs#nix-direnv
echo 'direnv hook fish | source' >> ~/.config/fish/config.fish
# または bash の場合: echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
direnv allow
```

---

## 2. ビルド

### 2.1 フルビルド (推奨)

```bash
# Nix シェル内で実行
cd main_ws

# フルビルド (Rust バインディング生成 → 全パッケージ)
just forge
```

`just forge` は以下を順に実行する:

1. `rosidl_generator_rs` をビルド (Rust メッセージバインディング生成器)
2. `install/setup.bash` を読み込み
3. 全パッケージを colcon でビルド (`--merge-install --symlink-install -DROSIDL_GENERATOR_RUST=ON -G Ninja`)

### 2.2 クリーンビルド

```bash
cd main_ws
just reforge   # build/ install/ log/ を削除してからフルビルド
```

### 2.3 個別パッケージビルド

```bash
cd main_ws
source install/setup.bash

# 特定パッケージだけビルド
colcon build --merge-install --symlink-install \
  --cmake-args -DROSIDL_GENERATOR_RUST=ON -G Ninja \
  --packages-select <パッケージ名>

# 例: arm_controller だけビルド
colcon build --merge-install --symlink-install \
  --cmake-args -DROSIDL_GENERATOR_RUST=ON -G Ninja \
  --packages-select arm_controller

# 例: bringup (launch/config/URDF) だけビルド (Rust 不要)
colcon build --merge-install --symlink-install \
  --packages-select bringup
```

### 2.4 Rust パッケージのみビルド

```bash
cd main_ws

# Rust ワークスペースのみ (colcon 経由でなく直接 cargo)
# ※ 事前に rosidl バインディングが生成されている必要あり
source install/setup.bash
cargo build --release
```

### 2.5 ビルド時の注意事項

| 項目 | 説明 |
|------|------|
| リンカ | Nix 環境では `clang` + `mold` を使用 (`Justfile` で自動設定) |
| `kiss_icp` | ビルドから除外されている (`--packages-ignore kiss_icp`) |
| `symlink-install` | Python スクリプト・launch ファイルはシンボリックリンクなので変更が即反映 |
| `LIBRARY_PATH` | `example_interfaces` のリンクに必要なパスは `flake.nix` で自動設定 |

### 2.6 overlay の読み込み

ビルド後、ROS 2 コマンドを使う前に必ず:

```bash
source install/setup.bash   # bash の場合
# fish の場合: bass source install/setup.bash
```

---

## 3. 実機起動

### 3.1 全ノード一括起動

```bash
cd main_ws
source install/setup.bash

# 全系統を一括起動
ros2 launch bringup system.launch.py
```

`system.launch.py` は以下を順に起動する:

| 順番 | ファイル | 内容 |
|------|----------|------|
| 1 | (robot_state_publisher) | URDF → TF 変換 |
| 2 | (foxglove_bridge) | Foxglove Studio 向け WebSocket ゲートウェイ (:8765) |
| 3 | `network.launch.py` | Zenoh ルーター (`--config zenoh_router.json5`) + RMW 環境変数 |
| 4 | `perception.launch.py` | LiDAR, IMU, SLAM |
| 5 | `camera.launch.py` | USB カメラ (usb_cam → /camera/image_raw) |
| 6 | `control.launch.py` | モータドライバ, コントローラ (joy_controller 含む), アーム, グリッパ |
| 7 | `nav2.launch.py` | 自律走行 (`use_nav2:=true` 時のみ) |

### 3.2 起動オプション

```bash
# SLAM 無効 + RViz 表示
ros2 launch bringup system.launch.py use_slam:=false use_rviz:=true

# Nav2 自律走行を有効化
ros2 launch bringup system.launch.py use_nav2:=true

# VLP-16 ドライバも起動（別マシンで動かす場合は false のまま）
ros2 launch bringup perception.launch.py use_velodyne:=true

# ダミー IMU（IMU なしでテストする場合）
ros2 launch bringup perception.launch.py use_dummy_imu:=true

# LiDAR deskew に問題がある場合 (Python 補正ノードを有効化)
ros2 launch bringup perception.launch.py use_time_fix:=true lidar_topic:=/velodyne_points_fixed
```

### 3.3 オペレータ側

オペレータ PC は **Foxglove Studio** と **joy_node** だけで動作する。
joy_controller は NUC (ロボット) 側の `control.launch.py` で起動される。
`/joy` トピックは Zenoh (QUIC) 経由で自動的に NUC に転送される。

#### 方法 A: operator/ ディレクトリの専用 flake を使う (推奨)

```bash
cd operator/

# 1. Zenoh 設定: <ROBOT_IP> を NUC の IP に書き換え
vim zenoh_ope.json5

# 2. Nix 環境に入る (ROS 2 + joy + Zenoh のみ、軽量)
nix develop --impure

# 3. PS4 コントローラを USB/Bluetooth 接続後
ros2 run joy joy_node
```

#### 方法 B: メインの Nix 環境を使う (開発マシン)

```bash
cd main_ws
nix develop --impure  # ルートの flake.nix
source install/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export RMW_ZENOH_CONFIG_URI=$(pwd)/src/bringup/config/zenoh_ope.json5
# ↑ <ROBOT_IP> を書き換え済みであること

ros2 run joy joy_node
```

#### Foxglove Studio での監視

Foxglove Studio をメインのオペレータ UI として使用する。
NUC 側の `system.launch.py` が `foxglove_bridge` を自動起動するので、
オペレータ PC で Foxglove Studio から接続するだけでよい。

```
接続先: ws://<ROBOT_IP>:8765
```

Foxglove Studio は Nix 不要。https://foxglove.dev/download からダウンロードするか:

```bash
sudo snap install foxglove-studio        # Ubuntu
flatpak install flathub dev.foxglove.studio  # Flatpak
```

#### オペレータ側で見えるトピック

| トピック | 内容 |
|----------|------|
| `/image/compressed` | QR 検出付きカメラ映像 (JPEG, 低帯域) |
| `/camera/image_raw` | 生カメラ映像 |
| `/velodyne_points` | 3D 点群 |
| `/map` | 2D SLAM 地図 |
| `/scan` | 2D LaserScan |
| `/robot_description` | ロボットモデル (URDF) |
| `/joint_states` | アーム・フリッパ関節角 |
| `/gripper_status` | グリッパ状態 |
| `/qr_codes` | QR コード検出テキスト |
| TF ツリー | map → odom → base_link → ... |

### 3.4 個別サブシステム起動

```bash
# カメラのみ
ros2 launch bringup camera.launch.py

# ロボットモデル表示 (robot_state_publisher + joint_state_publisher_gui + RViz)
ros2 launch bringup display.launch.py

# RViz なし (nixGL がない環境で)
ros2 launch bringup display.launch.py use_rviz:=false

# アーム単体の URDF でモデル確認
ros2 launch bringup display.launch.py urdf:=sekirei.urdf

# オペレータ側 (joy_node + joy_controller 開発用一括起動)
ros2 launch joy_controller operator_launch.py
```

---

## 4. シミュレーション

### 4.1 概要

Gazebo Harmonic (gz sim 8) を使用。実機のハードウェアドライバは起動せず、Gazebo プラグインがセンサとオドメトリを提供する。`ros_gz_bridge` が Gazebo トピックを ROS 2 トピックに変換する。

| 実機コンポーネント | シミュレーション代替 |
|---------------------|---------------------|
| VLP-16 + velodyne_driver | Gazebo gpu_lidar → `ros_gz_bridge` → `/velodyne_points` |
| STM32 + sensor_gateway | Gazebo IMU sensor → `ros_gz_bridge` → `/imu/data` |
| USB カメラ + usb_cam | Gazebo camera sensor → `ros_gz_bridge` → `/camera/image_raw` |
| crawler_driver (Roboclaw) | Gazebo DiffDrive plugin ← `ros_gz_bridge` ← `/cmd_vel` |
| joy_controller → CrawlerVelocity | `crawler_vel_bridge.py` が Twist に変換して Gazebo へ |

### 4.2 シミュレーション起動

```bash
cd main_ws
source install/setup.bash

# 基本起動 (Gazebo GUI + SLAM + RViz)
ros2 launch bringup simulation.launch.py

# ヘッドレス起動 (GUI なし、CI やリモート環境向け)
ros2 launch bringup simulation.launch.py headless:=true

# SLAM なし (Gazebo + テレオペのみ)
ros2 launch bringup simulation.launch.py use_slam:=false

# Nav2 自律走行付き
ros2 launch bringup simulation.launch.py use_slam:=true use_nav2:=true

# RViz なし
ros2 launch bringup simulation.launch.py use_rviz:=false

# カスタムワールド
ros2 launch bringup simulation.launch.py world:=/path/to/custom.sdf
```

### 4.3 シミュレーション引数一覧

| 引数 | デフォルト | 説明 |
|------|-----------|------|
| `world` | `rescue_field.sdf` | ワールドファイルパス (.sdf) |
| `headless` | `false` | `true` で GUI なし (サーバーモード + EGL レンダリング) |
| `use_slam` | `true` | SLAM Toolbox を起動 |
| `use_nav2` | `false` | Nav2 自律走行スタックを起動 |
| `use_rviz` | `true` | RViz2 を起動 |

### 4.4 シミュレーションで起動されるノード

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| gazebo (gz sim) | ros_gz_sim | 物理シミュレータ本体 |
| robot_state_publisher | robot_state_publisher | URDF → TF 変換 |
| create (spawn) | ros_gz_sim | ロボットモデルを Gazebo に投入 |
| parameter_bridge | ros_gz_bridge | Gazebo ↔ ROS 2 トピック変換 |
| crawler_vel_bridge | bringup | CrawlerVelocity → Twist 変換 |
| joy_controller | joy_controller | PS4 → 指令値 (シミュレーション時はローカル起動) |
| arm_controller | arm_controller | IK → 目標位置 |
| arm_gz_bridge | bringup | JointState → per-joint cmd_pos (Gazebo 用) |
| pointcloud_to_laserscan | pointcloud_to_laserscan | 3D → 2D 変換 |
| slam_toolbox | slam_toolbox | 2D SLAM (オプション) |
| nav2 スタック | navigation2 | 自律走行 (オプション) |
| rviz2 | rviz2 | 可視化 (オプション) |

### 4.5 ros_gz_bridge トピックマッピング

| ROS 2 トピック | 方向 | Gazebo トピック | メッセージ型 |
|---------------|------|----------------|------------|
| `/cmd_vel` | ROS 2 ↔ Gazebo | `/cmd_vel` | Twist ↔ gz.msgs.Twist |
| `/odom` | Gazebo → ROS 2 | `/odom` | Odometry |
| `/velodyne_points` | Gazebo → ROS 2 | `/velodyne_points` | PointCloud2 |
| `/imu/data` | Gazebo → ROS 2 | `/imu/data` | Imu |
| `/camera/image_raw` | Gazebo → ROS 2 | `/camera/image_raw` | Image |
| `/camera/camera_info` | Gazebo → ROS 2 | `/camera/camera_info` | CameraInfo |
| `/joint_states` | Gazebo → ROS 2 | `/joint_states` | JointState |
| `/clock` | Gazebo → ROS 2 | `/clock` | Clock |

### 4.6 ワールドファイル

デフォルト `rescue_field.sdf` は 10m×10m の屋内レスキューフィールド:

- 外壁 4 面
- 仕切り壁 + ドア開口部
- L 字通路
- 瓦礫風障害物 3 個 (ボックス × 2, シリンダー × 1)
- スロープ (フリッパ動作確認用)

### 4.7 シミュレーション操作

```bash
# 別ターミナルでコントローラを接続
ros2 run joy joy_node

# トピック一覧を確認
ros2 topic list

# オドメトリを確認
ros2 topic echo /odom --once

# cmd_vel を手動で送信してロボットを動かす
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}' --rate 10
```

### 4.8 センサレンダリングについて

GPU がない環境 (SSH, CI 等) ではセンサ系プラグイン (`gz-sim-sensors-system`) がクラッシュする場合がある。
その場合はワールド SDF の sensors plugin をコメントアウトする (デフォルトではコメントアウト済み)。
物理シミュレーション・差動駆動・odometry・TF は GPU なしでも動作する。

GUI 起動時は nixGL 経由で:

```bash
# nixGL 経由で Gazebo GUI を使う場合
ros2 launch bringup simulation.launch.py headless:=false
# ※ flake.nix で alias gz="nixGL gz" が設定済み
```

---

## 5. コントローラ操作

PS4 (DualShock 4) コントローラで操作する。
3 つの動作モードがあり、ボタンで切り替える。

### 5.1 モード切替

| ボタン | モード | 説明 |
|--------|--------|------|
| **PS** | STOP | **緊急停止** (E-Stop): 全モータ停止、ラッチ式（ノード再起動まで解除不可） |
| **OPTIONS** | DRIVE | クローラ＋フリッパ操作 |
| **SHARE** | ARM | ロボットアーム操作 |

> 起動直後は **STOP** モード。必ず OPTIONS を押して DRIVE に切り替えてから走行する。
> モード切替ボタンはすべてエッジ検出 (押した瞬間のみ反応、長押し不要)。

### 5.2 DRIVE モード

#### クローラ（走行）

| 入力 | 動作 |
|------|------|
| 左スティック ↑↓ | 左クローラ 前進/後退 |
| 右スティック ↑↓ | 右クローラ 前進/後退 |

最大速度: **0.7 m/s**、デッドゾーン: **0.1**

> 左右独立操作（スキッドステア方式）。両スティックを同時に倒すと直進、片方だけで超信地旋回。

#### フリッパ

| 入力 | フリッパ | 方向 |
|------|----------|------|
| D-Pad ↑ | 左後フリッパ | + |
| D-Pad ↓ | 左後フリッパ | − |
| × ボタン | 右後フリッパ | + |
| □ ボタン | 右後フリッパ | − |
| L1 | 左前フリッパ | + |
| L2 (深く引く) | 左前フリッパ | − |
| R1 | 右前フリッパ | + |
| R2 (深く引く) | 右前フリッパ | − |

フリッパ速度: **1000** (定数)

> L2/R2 はアナログトリガー。90% 以上引き込んだ時点 (値 < -0.9) で入力として認識される。

### 5.3 ARM モード

| 入力 | 動作 | スケール |
|------|------|----------|
| 左スティック ↑↓ | 手先 前後 (X) | 0.3 m/s |
| 左スティック ←→ | 手先 左右 (Y) | 0.3 m/s |
| 右スティック ↑↓ | 手先 上下 (Z) | 0.3 m/s |
| 右スティック ←→ | 手先 Yaw 回転 | 0.5 rad/s |
| L2 / R2 | Roll 回転 (R2で正, L2で負) | 0.5 rad/s (50%以上で反応) |
| D-Pad ↑↓ | Pitch 回転 (↑で正, ↓で負) | 0.5 rad/s |

> ARM モード中はクローラ・フリッパの指令はゼロになる（安全措置）。

---

## 6. 自律走行 (Nav2)

### 6.1 起動

```bash
# SLAM + Nav2 を同時に起動
ros2 launch bringup system.launch.py use_nav2:=true

# シミュレーションの場合
ros2 launch bringup simulation.launch.py use_nav2:=true

# RViz の "2D Goal Pose" ツールで地図上にゴールを指定
```

### 6.2 動作概要

1. `slam_toolbox` が `/map` (OccupancyGrid) を生成
2. Nav2 が `/map` と `/scan` からコストマップを構築
3. `SmacPlanner2D` でグローバルパスを計画
4. `RegulatedPurePursuit` でパスに沿って走行 (最大 0.3 m/s)
5. `/cmd_vel` (Twist) → `crawler_driver` が差動駆動に変換

### 6.3 Nav2 パラメータ

| 項目 | 値 | 設定ファイル |
|------|------|-------------|
| ローカルコントローラ | RegulatedPurePursuit | `nav2_params.yaml` |
| グローバルプランナー | SmacPlanner2D | `nav2_params.yaml` |
| ロボット半径 | 0.35m | costmap 設定 |
| インフレーション半径 | 0.55m | costmap 設定 |
| 最大直進速度 | 0.3 m/s | controller 設定 |

### 6.4 注意事項

- Nav2 走行中もコントローラの PS ボタンで即時停止可能
- 目標到達後は自動停止
- 障害物検出時は自動減速・回避
- パラメータ調整は `bringup/config/nav2_params.yaml` で行う

---

## 7. シリアルデバイス設定

### 7.1 udev ルール

各デバイスが固定パスで認識されるよう udev ルールを設定する。
`/etc/udev/rules.d/99-robot.rules` に以下を追加:

```udev
# Roboclaw モータコントローラ
SUBSYSTEM=="tty", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2404", SYMLINK+="roboclaw", MODE="0666"

# STM32F4 (BNO055 IMU ゲートウェイ)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="stm32", MODE="0666"

# Dynamixel U2D2 (グリッパ/フリッパ)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="dynamixel", MODE="0666"

# Dynamixel U2D2 (アーム) — 複数 U2D2 がある場合はシリアル番号で区別
# SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{serial}=="<SERIAL>", SYMLINK+="ttyUSB_arm", MODE="0666"
```

> **注意**: `idVendor` / `idProduct` は実機に合わせて `lsusb` で確認すること。

ルール適用:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 7.2 デバイス対応表

| デバイス | デフォルトパス | ノード | ボーレート |
|----------|----------------|--------|-----------|
| Roboclaw | `/dev/roboclaw` | crawler_driver | 38400 |
| STM32 (BNO055) | `/dev/stm32` | sensor_gateway | 115200 |
| Dynamixel (アーム) | `/dev/ttyUSB_arm` | arm_driver | 115200 |
| Dynamixel (グリッパ) | `/dev/ttyUSB1` | gripper_driver | 115200 |
| Dynamixel (フリッパ) | `/dev/dynamixel` | flipper_driver | 115200 |

---

## 8. ネットワーク構成

### 8.1 通信方式

- **プロトコル**: Zenoh (`rmw_zenoh_cpp`)
- **トランスポート**: QUIC (UDP ベース、Wi-Fi 向き)
- **ポート**: 7447
- **マルチキャスト**: 無効 (`--no-multicast-scout`)

### 8.2 接続構成

```
[ロボット NUC]                         [オペレータ PC]
zenohd (--config zenoh_router.json5)     ROS 2 ノード (zenoh_ope.json5)
  listen: quic/0.0.0.0:7447               connect: quic/<ROBOT_IP>:7447
  ↕                                       ↕
ROS 2 ノード群                            joy_node
  (zenoh_robot.json5)                    Foxglove Studio
foxglove_bridge (:8765) ----WebSocket---→ ws://<ROBOT_IP>:8765
```

### 8.3 Zenoh 設定ファイル

| ファイル | 用途 |
|----------|------|
| `bringup/config/zenoh_router.json5` | zenohd ルーターデーモン (router, listen quic/0.0.0.0:7447, SHM 有効) |
| `bringup/config/zenoh_robot.json5` | ロボット側 ROS 2 ノード (RMW_ZENOH_CONFIG_URI, client, quic/localhost:7447, SHM 有効) |
| `bringup/config/zenoh_ope.json5` | オペレータ側 ROS 2 ノード (client, quic/\<ROBOT_IP\>:7447) |

---

## 9. 本番デプロイ

### 9.1 構成概要

```
[オペレータ PC]                          [NUC (ロボット)]
  Foxglove Studio (ダウンロード)           Nix + flake.nix
  joy_node (operator/flake.nix)            system.launch.py (全ノード)
      │                                       │
      └── Zenoh QUIC (/joy) ──────────────────►├── joy_controller
          ws://<IP>:8765 ◄────────────────────├── foxglove_bridge
                                               ├── perception (LiDAR, SLAM)
                                               ├── control (arm, crawler, flipper, gripper)
                                               └── camera (usb_cam → qr_detector)
```

| 側 | 必要なもの | Nix 必要? |
|----|-----------|-----------|
| **オペレータ** | Foxglove Studio + joy_node + Zenoh | `operator/flake.nix` (軽量) |
| **NUC (ロボット)** | 全ノード + ドライバ + SLAM + Nav2 | `flake.nix` (フル) |

### 9.2 NUC (ロボット) セットアップ

#### 自動セットアップ

```bash
# NUC に SSH で入る
ssh user@<NUC_IP>

# リポジトリをクローン
git clone --recursive <REPO_URL>
cd rustacean_roborescue

# セットアップスクリプトを実行 (Nix インストール → ビルド → udev → systemd)
./deploy/setup-nuc.sh
```

`setup-nuc.sh` が行うこと:

| ステップ | 内容 |
|----------|------|
| 1 | Nix インストール (未インストール時) |
| 2 | Git サブモジュール取得 |
| 3 | Nix 環境プリビルド (初回 30-60 分) |
| 4 | `cd main_ws && just forge` (ROS 2 ワークスペースビルド) |
| 5 | udev ルールインストール (`deploy/99-robot.rules`) |
| 6 | systemd サービスインストール (`deploy/roborescue.service`) |

#### 手動セットアップ

```bash
# 1. Nix インストール
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install

# 2. クローン + ビルド
git clone --recursive <REPO_URL>
cd rustacean_roborescue
nix develop --impure
cd main_ws && just forge

# 3. udev ルール
sudo cp deploy/99-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 4. 手動起動
source install/setup.bash
ros2 launch bringup system.launch.py
```

### 9.3 NUC 自動起動 (systemd)

```bash
# サービス有効化 (NUC 起動時に自動で全ノードが立ち上がる)
sudo systemctl enable roborescue

# 手動で起動/停止
sudo systemctl start roborescue
sudo systemctl stop roborescue

# ログ確認
journalctl -u roborescue -f

# サービス状態
systemctl status roborescue
```

systemd サービスの動作:
- `deploy/launch-robot.sh` → `nix develop --impure` で環境に入り → `ros2 launch bringup system.launch.py`
- `SIGINT` でグレースフルシャットダウン (ros2 launch が Ctrl+C と同じ処理)
- クラッシュ時は 5 秒後に自動再起動 (`Restart=on-failure`)
- ログは `journalctl` に集約

### 9.4 オペレータ PC セットアップ

オペレータ側は **Foxglove Studio** と **joy_node** のみ。ビルド不要。

```bash
# 1. Foxglove Studio インストール (Nix 不要)
#    https://foxglove.dev/download
#    または: sudo snap install foxglove-studio

# 2. joy_node 用の軽量 Nix 環境
cd operator/

# 3. Zenoh 設定: <ROBOT_IP> を NUC の実 IP に書き換え
vim zenoh_ope.json5

# 4. 起動
nix develop --impure
ros2 run joy joy_node

# 5. Foxglove Studio で接続
#    ws://<ROBOT_IP>:8765
```

> **Nix なしのオペレータ**:  Ubuntu に ROS 2 Jazzy を apt インストールし、`rmw_zenoh_cpp` パッケージを追加すれば Nix なしでも動作する。ただし環境再現性の保証はない。

### 9.5 deploy/ ディレクトリ構成

| ファイル | 用途 |
|----------|------|
| `setup-nuc.sh` | NUC ワンショットセットアップ (Nix + ビルド + udev + systemd) |
| `launch-robot.sh` | systemd から呼ばれる起動スクリプト |
| `roborescue.service` | systemd ユニットファイル (テンプレート) |
| `99-robot.rules` | udev ルール (シリアルデバイス固定パス) |

### 9.6 本番チェックリスト

| # | 項目 | 確認方法 |
|---|------|----------|
| 1 | NUC に Nix がインストールされている | `nix --version` |
| 2 | サブモジュールが取得されている | `git submodule status` (先頭に `-` がないこと) |
| 3 | ビルドが成功している | `main_ws/install/` が存在する |
| 4 | udev ルールが適用されている | `ls -la /dev/roboclaw /dev/stm32 /dev/dynamixel` |
| 5 | シリアルデバイスにアクセスできる | `groups` に `dialout` が含まれている |
| 6 | NUC の IP が固定されている | `ip addr` で確認 |
| 7 | zenoh_ope.json5 の IP が正しい | オペレータ側で確認 |
| 8 | systemd サービスが動作する | `sudo systemctl start roborescue && journalctl -u roborescue -f` |
| 9 | Foxglove で接続できる | `ws://<NUC_IP>:8765` でトピックが見える |
| 10 | joy_node の `/joy` が NUC に届く | NUC 側で `ros2 topic echo /joy --once` |

---

## 10. トラブルシューティング

### 10.1 ビルドエラー

| 症状 | 原因 | 対処 |
|------|------|------|
| `Argument list too long` | Nix の gcc collect2 制限 | `Justfile` の clang+mold 設定を確認 |
| `example_interfaces` リンクエラー | `LIBRARY_PATH` 未設定 | `flake.nix` に `ros.example-interfaces` があるか確認 |
| Rust バインディングが見つからない | rosidl 未生成 | `just forge-bindings` を先に実行 |
| `nalgebra` バージョン競合 | k crate が 0.30 を要求 | `k::nalgebra` を使用 (standalone nalgebra を入れない) |

### 10.2 シミュレーション

| 症状 | 原因 | 対処 |
|------|------|------|
| `Could not initialize GLX` | Gazebo/RViz2 GUI に GPU がない | `headless:=true` で起動、または `nixGL rviz2` / `nixGL gz sim` を使用 |
| `SensorsPrivate::WaitForInit` SEGV | レンダリングエンジン初期化失敗 | SDF の sensors-system plugin をコメントアウト |
| `urdf_parser_plugin not found` | パッケージ不足 | `flake.nix` に `ros.urdf` + `ros.urdf-parser-plugin` 追加 |
| Entity creation timeout | Gazebo が起動前にスポーン | Gazebo 起動を待ってから再実行 |

### 10.3 Foxglove / ネットワーク

| 症状 | 原因 | 対処 |
|------|------|------|
| Foxglove が接続できない | foxglove_bridge 未起動 | `system.launch.py` が正常起動しているか確認 |
| Foxglove でトピックが見えない | WebSocket 接続先が違う | `ws://<ROBOT_IP>:8765` で接続しているか確認 |
| Zenoh のトピックが対向に届かない | プロトコル不一致 | zenoh_ope.json5 / zenoh_robot.json5 が共に QUIC になっているか確認 |
| nixGL がない環境で RViz2 が落ちる | OpenGL ドライバ不一致 | `nixGL rviz2` で起動、または `use_rviz:=false` で無効化 |

### 10.4 実機

| 症状 | 原因 | 対処 |
|------|------|------|
| シリアルポートが開けない | 権限不足 | udev ルール設定 + `MODE="0666"` |
| Dynamixel 通信エラー | ボーレート不一致 | `Dynamixel Wizard` でモータ設定確認 |
| IMU データが来ない | STM32 未接続 / ファームウェア未書込 | `stm32_ws` のファームウェアを焼く |
| LiDAR deskew が不正 (点群が歪む) | `timestamp_unit` 設定不正 | `spark_fast_lio_min.yaml` の `timestamp_unit: 0` (SEC) を確認。改善しない場合は `use_time_fix:=true lidar_topic:=/velodyne_points_fixed` で Python 補正ノードを有効化 |
| コントローラが効かない | STOP モード | OPTIONS ボタンで DRIVE モードに切替 |
| E-Stop 後に操作不能 | E-Stop はラッチ式（意図的仕様） | 全ノードを再起動する（下記参照） |

### 10.5 E-Stop 復帰手順

E-Stop（PS ボタン）は安全のため **ラッチ式** であり、ボタン操作では解除できない。
以下の手順で全系統を再起動する:

```bash
# 1. system.launch.py を Ctrl+C で停止
# 2. すべてのモータが停止していることを目視確認
# 3. 再起動
ros2 launch bringup system.launch.py
# 4. OPTIONS ボタンで DRIVE モードに切替
```

> **重要**: 再起動前に E-Stop の原因を確認し、問題が解消していることを確認すること。
