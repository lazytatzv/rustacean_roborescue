# Roboclaw Driver for ROS2

## 概要
このパッケージは、Roboclaw モータードライバを ROS2 で制御するためのノードを提供します。同期シリアル通信を用いてモーターの速度制御やエンコーダーのリセットを行います。ウォッチドッグタイマーにより、一定時間指令が途絶えた場合は自動停止します。

## 必要な依存関係

- ROS2 Humble 以降
- Boost Asio (`boost::asio`)
- `rclcpp`
- `custom_interfaces`（`custom_interfaces/msg/CrawlerVelocity`）

## ビルド方法

```sh
colcon build --packages-select crawler_driver
source install/setup.bash
```

## 使用方法

ノードを起動します。

```sh
ros2 run crawler_driver crawler_driver_node
```

`/crawler_driver` トピックに速度コマンドを送信すると、モーターが動作します。

```sh
ros2 topic pub /crawler_driver custom_interfaces/msg/CrawlerVelocity "{m1_vel: 1.0, m2_vel: 1.0}"
```

ウォッチドッグにより、指令が `watchdog_timeout_ms` ミリ秒以上途絶えるとモーターが自動停止します。

## トピック

| トピック名         | 型                                        | 説明                            |
|--------------------|-------------------------------------------|---------------------------------|
| `/crawler_driver`  | `custom_interfaces/msg/CrawlerVelocity`   | クローラ速度指令をSubscribe     |

## パラメータ

| パラメータ名            | デフォルト値    | 説明                                |
|-------------------------|-----------------|-------------------------------------|
| `serial_port`           | `/dev/roboclaw` | シリアルポートのパス                |
| `crawler_circumference` | `0.39`          | クローラーの円周（m）               |
| `counts_per_rev`        | `256`           | 1回転あたりのエンコーダーカウント数 |
| `gearhead_ratio`        | `66`            | 減速機の比率                        |
| `pulley_ratio`          | `2`             | プーリーの比率                      |
| `watchdog_timeout_ms`   | `500`           | ウォッチドッグタイムアウト（ms）    |

パラメータは YAML ファイルまたは起動時に指定できます。

```sh
ros2 run crawler_driver crawler_driver_node --ros-args -p serial_port:=/dev/ttyACM0
```

## ライセンス
MIT
