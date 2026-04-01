# Rustacean RoboRescue — Rescue Robot Control Platform

Rustacean RoboRescue は、屋内救助・探索タスク向けの ROS 2 ベース統合ロボット制御プラットフォームです。Rust / C++ / Python を混ぜたワークスペース構成で、実機・シミュレーション・オペレータ用ノードをまとめて管理します。

## Repository Layout

| Path | Purpose |
|------|---------|
| [main_ws/](main_ws) | Robot-side ROS 2 workspace. Drivers, bringup, control, perception, and vendored dependencies live here. |
| [operator_ws/](operator_ws) | Operator PC workspace. Zenoh client and operator launch files live here. |
| [stm32_ws/](stm32_ws) | STM32 firmware workspace for the IMU bridge and embedded support code. |
| [docs/](docs) | Architecture, operation, package inventory, and setup guides. |
| [deploy/](deploy) | NUC deployment scripts, udev rules, and systemd units. |

## What The Stack Covers

- 実機とシミュレーションの両方で動く bringup / launch 群
- クローラ、フリッパ、アーム、IMU のハードウェアドライバ
- 低遅延音声転送とカメラ映像のブリッジ
- Nav2 / SLAM / LiDAR / QR 検出 / 外部センサの統合
- Zenoh ベースの operator 連携と Foxglove 監視

## Quick Start

詳細は [docs/DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md) を参照してください。

```bash
# 1. 初回のみビルド
cd main_ws
just forge
cd ..

# 2. 実機起動 (1コマンド)
just robot-up

# 3. 通信確認用の最小起動 (トラブル時)
just robot-up-min
```

オペレータ側は [operator_ws/README.md](operator_ws/README.md) と [docs/OPERATION.md](docs/OPERATION.md) を参照してください。

## Package Guide

workspace 内の package 一覧と役割は [docs/PACKAGES.md](docs/PACKAGES.md) にまとめています。

## Documentation

| Document | Purpose |
|----------|---------|
| [docs/PACKAGES.md](docs/PACKAGES.md) | Workspace package inventory and entry points |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | System architecture, node graph, topics, TF, and data flow |
| [docs/OPERATION.md](docs/OPERATION.md) | Build, run, deploy, and troubleshooting guide |
| [docs/DEV_QUICKSTART.md](docs/DEV_QUICKSTART.md) | Fast setup guide for Nix / Docker / Just |
| [docs/NIX.md](docs/NIX.md) | Nix flake and dev-shell details |

## Hardware Map

| Component | Interface | Driver |
|-----------|-----------|--------|
| Crawler motors (Roboclaw) | UART `/dev/roboclaw` | `crawler_driver` |
| Flippers (Dynamixel XM) | RS485 `/dev/ttyUSB_flipper` | `flipper_driver` |
| 6-DOF arm + gripper (Dynamixel XM) | RS485 `/dev/ttyUSB_arm` | `arm_driver` |
| IMU (BNO055 via STM32) | UART `/dev/stm32` | `sensor_gateway` |
| Camera (USB / UVC / RealSense depending on the setup) | `/dev/video*` | `v4l2_camera`, `qr_detector`, `h264_republisher` |
| Audio I/O | PulseAudio / GStreamer | `audio_bridge` |
| LiDAR (real robot) | Ethernet | vendor LiDAR driver, pointcloud bridge, SLAM stack |

## License

Apache-2.0. See [LICENSE](LICENSE) for details.
