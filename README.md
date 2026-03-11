# Rustacean RoboRescue

ROS 2 Jazzy ベースのレスキューロボット制御システム。
Rust (rclrs) + C++ (rclcpp) + Python (rclpy) のマルチ言語ワークスペース。

## 特徴

- **ROS 2 Jazzy Jalisco** — Rust バインディング (rclrs 0.7) による高信頼ドライバ
- **6-DOF アーム制御** — Jacobian ベース速度 IK (適応 DLS + 特異点回避 + 関節リミット)
- **Nav2 自律走行** — SLAM Toolbox + SmacPlanner2D + RegulatedPurePursuit (0.3 m/s)
- **Nix 再現ビルド** — `nix develop --impure` で全依存を再現
- **Zenoh (QUIC)** — rmw_zenoh_cpp による WAN 対応通信
- **Gazebo Harmonic** — シミュレーション環境 (ros_gz_bridge 連携)
- **Foxglove Studio** — WebSocket (:8765) によるリモート可視化
- **本番デプロイ** — systemd サービス + udev ルール + ワンショットセットアップスクリプト

## アーキテクチャ

```text
Operator PC                              Robot (NUC)
┌────────────────────┐  Zenoh/QUIC   ┌──────────────────────┐
│ Foxglove Studio    │◄──────────────►│ joy_controller       │
│ joy (PS4)          │               │ arm_controller (IK)  │
└────────────────────┘               │ arm_driver (Dxl)     │
                                     │ gripper_driver (Dxl) │
                                     │ crawler_driver (RC)  │
                                     │ flipper_driver (Dxl) │
                                     │ sensor_gateway (IMU) │
                                     │ qr_detector (CV)     │
                                     └──────────────────────┘
```

詳細: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)

## クイックスタート

```bash
# 1. Nix 開発環境に入る
nix develop --impure

# 2. ワークスペースをビルド
cd main_ws
just forge

# 3a. シミュレーション起動
source install/setup.bash
ros2 launch bringup simulation.launch.py

# 3b. Nav2 自律走行付きシミュレーション
ros2 launch bringup simulation.launch.py use_nav2:=true

# 3c. 実機起動
source install/setup.bash
ros2 launch bringup system.launch.py
```

## 開発

```bash
cd main_ws

# コード品質チェック (format + lint + test)
just check

# フォーマット (自動修正)
just fmt

# コンパイルチェック (高速)
just check-compile

# テスト実行
just test
```

## ドキュメント

| ファイル | 内容 |
|---------|------|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | システム構成・ノード詳細・トピック一覧・データフロー |
| [OPERATION.md](docs/OPERATION.md) | ビルド・起動・シミュレーション・デプロイ・トラブルシューティング |
| [NIX.md](docs/NIX.md) | Nix 環境の詳細設定 |
| [main_ws/README.md](main_ws/README.md) | ROS 2 ワークスペース構成・パッケージ一覧 |

## ハードウェア

| コンポーネント | インターフェース | ドライバ |
|---------------|-----------------|---------|
| クローラモータ (Roboclaw) | UART `/dev/roboclaw` | crawler_driver (C++) |
| フリッパ (Dynamixel XM) | UART `/dev/ttyUSB_flipper` | flipper_driver (C++) |
| 6-DOF アーム (Dynamixel XM) | UART `/dev/ttyUSB_arm` | arm_driver (Rust) |
| グリッパ (Dynamixel XM) | UART `/dev/ttyUSB1` | gripper_driver (Rust) |
| IMU (BNO055 via STM32) | UART `/dev/stm32` | sensor_gateway (Rust) |
| LiDAR (Velodyne VLP-16) | Ethernet | velodyne_driver |
| カメラ (USB) | USB | usb_cam |

## ライセンス

Apache License 2.0 — [LICENSE](LICENSE) を参照。
