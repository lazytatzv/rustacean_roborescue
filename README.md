# Rustacean RoboRescue — Rescue Robot Control Platform

<!-- Badges: keep these placeholders; replace with organization/repo as needed -->
![CI](https://github.com/yourorg/yourrepo/workflows/CI/badge.svg)
![Codecov](https://codecov.io/gh/yourorg/yourrepo/branch/main/graph/badge.svg)
![License](https://img.shields.io/badge/license-Apache%202.0-blue)

Rustacean RoboRescue は、屋内救助・探索タスク向けの統合ロボット制御プラットフォームです。
堅牢なロボティクスミドルウェア（ROS 2）を基盤に、Rust/C++/Python を組み合わせたマルチランゲージワークスペースで実装されています。

目的:
- 実機とシミュレーションの双方で再現可能なロボットソフトウェアスタックの提供
- 高信頼なローカル制御（アーム、グリッパ、クローラ、フリッパ）と低遅延テレオペレーション
- コンテスト（RoboCup Rescue など）を念頭に置いた高速な運用と診断性

主要機能（抜粋）
- 6-DOF アーム制御（IK + DLS-based damping／関節制限）
- Dynamixel ベースのドライバ（クローラ、フリッパ、アーム、グリッパ）
- WebRTC/GStreamer を用いた低遅延映像・音声転送と ROS2 ベースのシグナリング
- Nix による再現性の高い開発シェルと Docker ベースの代替ワークフロー

クイックスタート（詳細は docs/DEV_QUICKSTART.md を参照）

1. 開発シェル（推奨：Nix）

```bash
# リポジトリのルートで
nix develop --accept-flake-config
```

2. ワークスペースの準備

```bash
cd main_ws
just forge      # 初回セットアップ（依存生成など）
colcon build --merge-install
source install/setup.bash
```

3. シミュレーション実行（例）

```bash
ros2 launch bringup simulation.launch.py
ros2 launch bringup simulation.launch.py use_nav2:=true  # Nav2 を有効にする場合
```

開発ワークフロー

```bash
# コード品質チェック（フォーマット、lint、quick tests）
just check

# 自動フォーマット
just fmt

# テスト（ローカル）
just test
```

ドキュメント索引（詳細は `docs/README.md` を参照）

- ARCHITECTURE — システム構成とデータフロー: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- OPERATION — ビルド／デプロイ／運用手順とトラブルシューティング: [docs/OPERATION.md](docs/OPERATION.md)
- NIX — Nix フレークと dev-shell の詳細: [docs/NIX.md](docs/NIX.md)
- DEV_QUICKSTART — 開発者向け即席手順: [docs/DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md)

ハードウェア構成（抜粋）

| コンポーネント | インターフェース | ドライバ |
|---------------|-----------------|---------|
| クローラモータ (Roboclaw) | UART `/dev/roboclaw` | `crawler_driver` (C++) |
| フリッパ (Dynamixel XM) | UART `/dev/ttyUSB_flipper` | `flipper_driver` (C++) |
| 6-DOF アーム (Dynamixel XM) | UART `/dev/ttyUSB_arm` | `arm_driver` (Rust) |
| グリッパ (Dynamixel XM) | UART `/dev/ttyUSB1` | `gripper_driver` (Rust) |
| IMU (BNO055 via STM32) | UART `/dev/stm32` | `sensor_gateway` (Rust) |
| LiDAR (Velodyne VLP-16) | Ethernet | `velodyne_driver` |
| カメラ (USB) | USB | `usb_cam` / v4l2 |

ライセンス: Apache-2.0 — 詳細は [LICENSE](LICENSE)

貢献: 開発者向けガイドラインは [CONTRIBUTING.md](CONTRIBUTING.md) を参照してください。

---
修正履歴: ドキュメントの改善を継続します。追加で目次・テンプレート・リンクチェックを行い、必要なら PR を分割して提出します。
