# Rustacean RoboRescue — Rescue Robot Control Platform

Rustacean RoboRescue は、屋内救助・探索タスク向けの統合ロボット制御プラットフォームです。
堅牢なロボティクスミドルウェア（ROS 2）を基盤に、Rust/C++/Python を組み合わせたマルチランゲージワークスペースで実装されています。

目的:
- 実機とシミュレーションの双方で再現可能なロボットソフトウェアスタックの提供
- 高信頼なローカル制御（アーム、グリッパ、クローラ、フリッパ）と低遅延テレオペレーション
- コンテスト（RoboCup Rescue など）を念頭に置いた高速な運用と診断性

主要機能（抜粋）:
- 6-DOF アーム制御（IK + DLS-based damping／関節制限）
- Dynamixel ベースのドライバ（クローラ、フリッパ、アーム、グリッパ）
- v4l2_camera + ffmpeg_image_transport による低遅延 H.264 映像転送
- Foxglove Studio によるリアルタイム映像・センサ監視
- Nix による再現性の高い開発シェル

## アーキテクチャ

```text
Operator PC                                Robot (NUC)
┌──────────────────────────┐  Zenoh/QUIC  ┌──────────────────────────┐
│ Foxglove Studio          │◄─────────────►│ joy_controller           │
│ joy_node (PS4)           │  /joy →       │ arm_controller (IK)      │
│ foxglove_bridge (:8765)  │  ← /camera   │ arm_driver (Dynamixel)   │
└──────────────────────────┘               │ crawler_driver (Roboclaw)│
                                           │ flipper_driver (Dxl)     │
                                           │ sensor_gateway (IMU)     │
                                           │ v4l2_camera + QR検出     │
                                           │ zenohd router (:7447)    │
                                           └──────────────────────────┘
```

## クイックスタート

詳細は [docs/DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md) を参照。

### 1. 開発シェル（推奨：Nix）

```bash
# リポジトリのルートで
nix develop --accept-flake-config
```

### 2. ワークスペースのビルド

```bash
cd main_ws
just forge      # 初回セットアップ（依存生成など）
colcon build --merge-install
source install/setup.bash
```

### 3. 実機起動

**ロボット側 (NUC)**:
```bash
cd main_ws
source install/setup.bash
ros2 launch bringup system.launch.py
```

**オペレータ側 (ラップトップ)**:
```bash
# operator_ws/zenoh_ope.json5 の Robot IP を確認・更新してから
cd operator_ws
ros2 launch launch/operator.launch.py
# Foxglove Studio で ws://127.0.0.1:8765 に接続
```

### 4. シミュレーション実行

```bash
ros2 launch bringup simulation.launch.py
ros2 launch bringup simulation.launch.py use_nav2:=true  # Nav2 を有効にする場合
```

## 開発ワークフロー

```bash
# コード品質チェック（フォーマット、lint、quick tests）
just check

# 自動フォーマット
just fmt

# テスト
just test
```

## ドキュメント

詳細は `docs/README.md` を参照。

| ドキュメント | 内容 |
|-------------|------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | システム構成・ノード・トピック・TF ツリー・データフロー |
| [docs/OPERATION.md](docs/OPERATION.md) | ビルド・デプロイ・操作・トラブルシューティング |
| [docs/DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md) | 開発者向け即席手順（Nix/Docker） |
| [docs/NIX.md](docs/NIX.md) | Nix フレークと dev-shell の詳細 |

## ハードウェア構成

| コンポーネント | インターフェース | ドライバ |
|---------------|-----------------|---------|
| クローラモータ (Roboclaw) | UART `/dev/roboclaw` | crawler_driver (C++) |
| フリッパ (Dynamixel XM) | RS485 `/dev/ttyUSB_flipper` | flipper_driver (C++) |
| 6-DOF アーム + グリッパ (Dynamixel XM) | RS485 `/dev/ttyUSB_arm` | arm_driver (Rust) |
| IMU (BNO055 via STM32) | UART `/dev/stm32` | sensor_gateway (Rust) |
| LiDAR (Velodyne VLP-16) | Ethernet (10.42.0.242:2368) | velodyne_driver |
| カメラ (USB) | `/dev/video0` | v4l2_camera |

## ライセンス

Apache-2.0 — 詳細は [LICENSE](LICENSE)

貢献: 開発者向けガイドラインは [CONTRIBUTING.md](CONTRIBUTING.md) を参照してください。
