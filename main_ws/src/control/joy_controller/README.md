# JoyController ノード

このNodeは、Joystickの入力を受け取り、クローラとフリッパーの速度指令を別々のメッセージで送信します。
ボタン操作により STOP / DRIVE モードを切り替えます。

## 必要な依存関係

- ROS2 Humble 以降
- `sensor_msgs` package
- `custom_interfaces` package（`CrawlerVelocity` msg, `FlipperVelocity` msg を含む）

## ビルド方法

```sh
colcon build --packages-select joy_controller
source install/setup.bash
```

## 使用方法

### ノードの実行

```sh
ros2 run joy_controller joy_controller_node
```

### ジョイスティック操作

- **Share ボタン** を押すと `STOP` モードに変更
- **Options ボタン** を押すと `DRIVE` モードに変更
- **左スティック Y軸** で左クローラの速度を制御
- **右スティック Y軸** で右クローラの速度を制御
- **D-Pad / L1,L2 / R1,R2 / Cross,Square** でフリッパーを制御

## トピック

| トピック名        | 型                                        | 説明                          |
|-------------------|-------------------------------------------|-------------------------------|
| `/joy`            | `sensor_msgs/msg/Joy`                     | Joystickの入力をSubscribe     |
| `/crawler_driver` | `custom_interfaces/msg/CrawlerVelocity`   | クローラ速度をPublish         |
| `/flipper_driver` | `custom_interfaces/msg/FlipperVelocity`   | フリッパー速度をPublish       |

## ライセンス

MIT

MIT License
