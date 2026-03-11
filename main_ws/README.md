# main_ws — ROS 2 ワークスペース

レスキューロボット「Rustacean RoboRescue」の ROS 2 Jazzy ワークスペース。
Rust (rclrs) + C++ (rclcpp) + Python (rclpy) のマルチ言語構成。

## パッケージ構成

### 制御系 (`src/control/`)

| パッケージ | 言語 | 概要 |
|-----------|------|------|
| `joy_controller` | C++ | PS4 コントローラ → 各ドライバへの指令変換 (STOP/DRIVE/ARM 3モード) |
| `arm_controller` | Rust | Jacobian ベース速度 IK (適応 DLS + 特異点回避 + 関節リミット) |

### ハードウェアドライバ (`src/hardware_drivers/`)

| パッケージ | 言語 | 概要 |
|-----------|------|------|
| `crawler_driver` | C++ | Roboclaw モータドライバ (差動駆動 + PID + `/cmd_vel` 対応) |
| `flipper_driver` | C++ | Dynamixel フリッパ (Wheel Mode + WatchDog 安全機構) |
| `arm_driver` | Rust | Dynamixel 6-DOF アームサーボ (Position Mode + E-Stop + 自動再接続) |
| `gripper_driver` | Rust | Dynamixel グリッパ (電流位置制御 + 把持状態マシン) |
| `sensor_gateway` | Rust | STM32 → BNO055 IMU パーサー (CSV → sensor_msgs/Imu + 自動再接続) |

### 認識系 (`src/perception/`)

| パッケージ | 言語 | 概要 |
|-----------|------|------|
| `qr_detector` | Python | WeChatQRCode による QR コード検出 + 圧縮画像配信 |
| `vision_processor` | Rust | QR コードデバッグツール (スタンドアロン、ROS ノードではない) |

### 統合 (`src/bringup/`)

| 内容 | 説明 |
|------|------|
| Launch ファイル | `system.launch.py`, `simulation.launch.py`, `nav2.launch.py`, `perception.launch.py`, `control.launch.py`, `network.launch.py`, `camera.launch.py`, `display.launch.py` |
| 設定 YAML | Nav2, SLAM, Velodyne, spark_fast_lio, ノード別パラメータ等 |
| URDF/Xacro | `robot.urdf.xacro` (シミュレーション用), `sekirei.urdf` (アーム IK 用) |
| Gazebo SDF | `rescue_field.sdf` (10×10m レスキューフィールド) |
| RViz Config | `simulation.rviz` (Nav2 + SLAM 表示), `display.rviz` (モデル表示) |
| ブリッジスクリプト | `arm_gz_bridge.py`, `odom_tf_bridge.py`, `crawler_vel_bridge.py`, `dummy_imu_node.py`, `fix_pointcloud_time_node.py` |
| Zenoh 設定 | `zenoh_router.json5`, `zenoh_robot.json5`, `zenoh_ope.json5` |

### カスタムメッセージ (`src/custom_interfaces/`)

| メッセージ | 用途 |
|-----------|------|
| `CrawlerVelocity.msg` | 左右クローラ速度 (m1_vel, m2_vel) |
| `FlipperVelocity.msg` | 4 フリッパ速度配列 (flipper_vel[]) |
| `GripperCommand.msg` | グリッパ目標位置 + 電流制限 |
| `GripperStatus.msg` | グリッパ状態 (IDLE/MOVING/GRIPPING/OVERLOAD/ERROR) |

### 外部サブモジュール (`src/external/`)

| サブモジュール | 用途 |
|---------------|------|
| `ros2_rust/` | rclrs — Rust 用 ROS 2 クライアントライブラリ |
| `rosidl_rust/` | rosidl_generator_rs — Rust メッセージ生成器 |
| `spark-fast-lio/` | FAST-LIO2 ベース LiDAR 慣性オドメトリ |
| `common_interfaces/` | 標準 ROS 2 メッセージ (std_msgs, sensor_msgs 等) |
| `rcl_interfaces/` | ROS 2 パラメータ/サービスインターフェース |

## ビルド

```bash
# Nix 環境に入る
nix develop --impure

# フルビルド
cd main_ws
just forge

# Rust バインディングのみ再生成
just forge-bindings

# コード品質チェック (format + lint + test)
just check

# テスト
just test
```

## 起動

```bash
source install/setup.bash

# 実機起動 (全ノード)
ros2 launch bringup system.launch.py

# 自律走行付き
ros2 launch bringup system.launch.py use_nav2:=true

# シミュレーション
ros2 launch bringup simulation.launch.py

# Nav2 付きシミュレーション
ros2 launch bringup simulation.launch.py use_nav2:=true
```

## 設計方針

- **低レイヤーは Rust**: ハードウェアドライバ (arm_driver, gripper_driver, sensor_gateway) は Rust (rclrs) で型安全・メモリ安全に実装
- **リアルタイム制御は C++**: クローラ/フリッパドライバ、コントローラは C++ (rclcpp) で実装
- **認識系は Python**: QR 検出は OpenCV + Python で実用性重視
- **SLAM/オドメトリは C++ 既存ライブラリ**: spark_fast_lio (FAST-LIO2), slam_toolbox

詳細は [docs/ARCHITECTURE.md](../docs/ARCHITECTURE.md) を参照。
