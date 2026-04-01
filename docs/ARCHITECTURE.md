# システムアーキテクチャ

## 目次

1. [プロジェクト概要](#1-プロジェクト概要)
2. [ディレクトリ構成](#2-ディレクトリ構成)
3. [パッケージ詳細](#3-パッケージ詳細)
4. [通信アーキテクチャ](#4-通信アーキテクチャ)
5. [トピック構成](#5-トピック構成)
6. [TF ツリー](#6-tf-ツリー)
7. [Launch 構成](#7-launch-構成)
8. [設定ファイル一覧](#8-設定ファイル一覧)
9. [データフロー図](#9-データフロー図)
10. [外部サブモジュール](#10-外部サブモジュール)

---

## 1. プロジェクト概要

レスキューロボット「Rustacean RoboRescue」の ROS 2 制御ソフトウェア。

| 項目 | 内容 |
|------|------|
| ROS ディストロ | **Jazzy Jalisco** |
| 言語 | Rust (制御・ドライバ), C++ (テレオペ・走行系), Python (認識・スクリプト) |
| ビルドシステム | colcon + cargo-ament-build + Ninja |
| 環境管理 | Nix flake (`--impure`) |
| RMW | rmw_zenoh_cpp (Zenoh / QUIC) |
| オペレータ UI | Foxglove Studio (foxglove_bridge 経由 WebSocket :8765) |
| シミュレータ | Gazebo Harmonic (gz sim 8) |
| ハードウェア | VLP-16 LiDAR, BNO055 IMU, Roboclaw, Dynamixel, USB カメラ |

---

## 2. ディレクトリ構成

```
rustacean_roborescue/
├── flake.nix                   # Nix 開発環境 (全依存定義, NUC 本番兼用)
├── Justfile                    # ルートコマンド (nix, sync)
├── docs/
│   ├── ARCHITECTURE.md         # 本ドキュメント
│   ├── OPERATION.md            # ビルド・起動・操作マニュアル
│   └── NIX.md                  # Nix セットアップ
├── operator_ws/                # オペレータ PC 用 ROS 2 ワークスペース
│   ├── launch/
│   │   └── operator.launch.py  # joy_node + foxglove_bridge を一括起動
│   ├── zenoh_ope.json5         # Zenoh 設定 (<ROBOT_IP> を書き換え)
│   └── README.md               # オペレータ セットアップ手順
├── deploy/                     # NUC 本番デプロイ用
│   ├── setup-nuc.sh            # ワンショットセットアップ
│   ├── launch-robot.sh         # systemd 起動スクリプト
│   ├── roborescue.service      # systemd ユニットファイル
│   └── 99-robot.rules          # udev ルール
├── main_ws/                    # ROS 2 メインワークスペース (ロボット側)
│   ├── Cargo.toml              # Rust ワークスペース定義
│   ├── Justfile                # ビルドコマンド (forge, reforge)
│   └── src/
│       ├── audio_bridge/       # Python: GStreamer/Opus 双方向音声通信
│       ├── bringup/            # Launch, Config, URDF, Scripts
│       ├── control/            # 制御系パッケージ
│       │   ├── arm_controller/     # Rust: 6DOF アーム IK (URDF/メッシュは bringup に統合)
│       │   └── joy_controller/     # C++: PS4 テレオペ (NUC 側で起動)
│       ├── custom_interfaces/  # カスタムメッセージ定義
│       ├── extras/             # 補助ユーティリティ
│       │   └── h264_republisher/   # C++: H.264 → sensor_msgs/Image 逆変換 (デバッグ用)
│       ├── hardware_drivers/   # ハードウェアドライバ
│       │   ├── arm_driver/         # Rust: Dynamixel アーム + グリッパ
│       │   ├── crawler_driver/     # C++: Roboclaw 走行モータ
│       │   ├── flipper_driver/     # C++: Dynamixel フリッパ
│       │   └── sensor_gateway/     # Rust: STM32 IMU ブリッジ
│       ├── perception/         # 認識系パッケージ
│       │   ├── omron_bridge/       # Python: OMRON 2JCIE-BU01 環境センサ
│       │   ├── qr_detector/        # Python: QR コード検出
│       │   ├── qr_detector_cpp/    # C++: QR コード検出 (WeChatQRCode, composable)
│       │   ├── robo_map_exporter/  # Python: SLAM マップ PLY/CSV エクスポート
│       │   └── vision_processor/   # Rust: OpenCV QR 検出 (実験的)
│       └── external/           # Git サブモジュール
└── stm32_ws/                   # STM32 ベアメタルファームウェア (STM32F446 + BNO055)
    └── src/main.rs
```

---

## 3. パッケージ詳細

### 3.1 `joy_controller` — PS4 テレオペレーション

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/control/joy_controller/` |
| 言語 | C++ |
| ビルド | ament_cmake |
| 依存 | rclcpp, sensor_msgs, geometry_msgs, custom_interfaces |

#### 概要

PS4 コントローラの入力を各アクチュエータの指令値に変換するテレオペノード。STOP / DRIVE / ARM の 3 モードを持ち、ボタンで切り替える。

#### トピック

| 方向 | トピック | 型 |
|------|---------|------|
| Subscribe | `/joy` | sensor_msgs/Joy |
| Publish | `/crawler_driver` | custom_interfaces/CrawlerVelocity |
| Publish | `/flipper_driver` | custom_interfaces/FlipperVelocity |
| Publish | `/arm_cmd_vel` | geometry_msgs/Twist (手先速度指令, IK 経由) |
| Publish | `/arm_joint_cmd_vel` | sensor_msgs/JointState (ジョイント直接速度指令) |
| Publish | `/gripper_cmd` | custom_interfaces/GripperCommand |
| Publish | `/emergency_stop` | std_msgs/Bool (transient_local, reliable) |

#### ROS パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `max_speed` | double | `0.7` | クローラ最大速度 [m/s] |
| `deadzone` | double | `0.1` | スティックデッドゾーン |
| `flipper_speed` | int | `1000` | フリッパ速度 [Dynamixel 単位] |
| `arm_linear_scale` | double | `0.3` | アーム並進速度スケール [m/s] |
| `arm_angular_scale` | double | `0.5` | アーム回転速度スケール [rad/s] |

#### 動作詳細

- **STOP モード** (PS ボタン): 全出力ゼロ + `/emergency_stop` (Bool{true}) を publish。全ドライバが即座にモーター停止。**ラッチ式**: 一度発動すると `estop_latched_` フラグが立ち、ノード再起動まで DRIVE/ARM への復帰不可。
- **DRIVE モード** (OPTIONS ボタン):
  - 左スティック Y → 左クローラ速度、右スティック Y → 右クローラ速度 (最大 0.7 m/s, デッドゾーン 0.1)
  - D-Pad, ×□, L1/L2/R1/R2 → フリッパ速度 ±1000
- **ARM モード** (SHARE ボタン):
  - 左スティック → XY 並進 (0.3 m/s)、右スティック → Z + Yaw (Z: 0.3 m/s, Yaw: 0.5 rad/s)
  - L2/R2 トリガー → Roll (0.5 rad/s)、D-Pad 上下 → Pitch (0.5 rad/s)

#### 安全機能

| 機能 | 説明 |
|------|------|
| ボタンエッジ検出 | 立ち上がりエッジのみで動作 (`prev_ps_`, `prev_options_`, `prev_share_` で前回状態を追跡)。ボタン長押しによるメッセージ洪水を防止 |
| E-Stop ラッチ | PS ボタンで発動後、`estop_latched_=true` が永続。DRIVE/ARM への切替を完全に遮断。解除はノード再起動のみ |
| アナログ閾値 | L2/R2 トリガーは `< -0.9f`、D-Pad は `< -0.5f` / `> 0.5f` で判定 (浮動小数点完全一致 `== -1.0f` を回避) |
| 設定ファイル | `bringup/config/joy_controller.yaml` から全パラメータをロード |

#### デプロイ配置

- **joy_controller**: NUC (ロボット側) で `control.launch.py` から起動
- **joy_node**: オペレータ PC で `operator_ws/launch/operator.launch.py` から起動
- `/joy` トピックは Zenoh 経由で NUC に転送される

---

### 3.2 `arm_controller` — 6DOF ロボットアーム逆運動学

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/control/arm_controller/` |
| 言語 | Rust |
| ビルド | ament_cargo |
| 依存 | rclrs, geometry_msgs, sensor_msgs, std_msgs, `k` (0.32), `urdf-rs` (0.8), anyhow |

#### 概要

6 自由度アームの手先速度指令を受け取り、Damped Least Squares (DLS) 法による速度逆運動学でジョイント速度に変換し、**内部で積分して目標位置を出力する**ノード。下流の `arm_driver` (実機・`arm_backend:=direct`) または `arm_cmd_bridge` → `ros2_control` (実機・`arm_backend:=ros2_control`) または `arm_gz_bridge` (Gazebo) が位置指令を受け取る。

#### 安全機能

| 機能 | 説明 |
|------|------|
| ジョイント位置フィードバック | `/joint_states` を購読し実際のエンコーダ値で FK 計算 |
| ジョイントリミット回避 | リミット近傍 (0.1 rad) で方向別に速度を減衰 + ヌル空間で中立点へ反発 |
| 適応 DLS ダンピング | マニピュラビリティ $w$ が低下すると $\lambda$ を自動増加 (Nakamura–Hanafusa 法) |
| 特異姿勢ロックアウト | $w < 0.0005$ で全速度ゼロ (危険姿勢での暴走防止) |
| E-Stop | `/emergency_stop` 受信時、全指令出力を停止 (transient_local QoS) |
| 起動安全 | `/joint_states` 受信前は指令を受け付けない。アーム全6軸のジョイント名がすべて含まれている JointState のみ受理 (フリッパの JointState で誤動作しない) |
| ウォッチドッグ | 500ms 無指令で全モータ停止 |
| 位置積分 | 速度を内部で積分し目標位置を出力 (ジョイントリミットでクランプ) |

#### ROS パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `urdf_path` | string | `""` (必須) | アーム URDF ファイルパス |
| `end_link` | string | `"link_tip"` | エンドエフェクタのリンク名 |
| `watchdog_timeout_ms` | int | `500` | 指令タイムアウト [ms] |
| `dls_lambda_base` | double | `0.01` | DLS ダンピング基本値 $\lambda_0$ |
| `dls_lambda_max` | double | `0.15` | 特異点近傍の最大ダンピング |
| `manipulability_threshold` | double | `0.005` | ダンピング増加開始閾値 |
| `manipulability_lockout` | double | `0.0005` | 動作停止閾値 |
| `joint_vel_limit` | double | `1.0` | ジョイント速度上限 [rad/s] |
| `joint_limit_margin` | double | `0.10` | リミット近傍減速マージン [rad] |
| `null_space_repulsion_gain` | double | `0.5` | ヌル空間反発ゲイン |

#### トピック

| 方向 | トピック | 型 |
|------|---------|------|
| Subscribe | `/arm_cmd_vel` | geometry_msgs/Twist (手先速度 → IK) |
| Subscribe | `/arm_joint_cmd_vel` | sensor_msgs/JointState (ジョイント直接速度) |
| Subscribe | `/joint_states` | sensor_msgs/JointState (position フィードバック) |
| Subscribe | `/emergency_stop` | std_msgs/Bool (transient_local, reliable) |
| Publish | `/arm_joint_commands` | sensor_msgs/JointState (.position = 目標位置, .velocity = 情報用) |
| Publish | `/arm_ee_pose` | geometry_msgs/PoseStamped (FK 結果, エンドエフェクタ姿勢) |

#### アルゴリズム

1. URDF を `k` クレートで読み込み、エンドエフェクタリンクまでの `SerialChain` を構築
2. `/joint_states` から実際のジョイント角度を取得し、FK を更新
3. 50Hz タイマーで `/arm_cmd_vel` の Twist を $\dot{x} \in \mathbb{R}^6$ に変換
4. マニピュラビリティ $w = \sqrt{\det(J J^T)}$ を計算、$w$ に応じて $\lambda$ を適応調整
5. DLS: $\dot{q}_{task} = J^T (J J^T + \lambda^2 I)^{-1} \dot{x}$
6. ヌル空間投影: $\dot{q}_{null} = (I - J^+ J) \cdot \nabla h(q)$ でジョイントを中立点へ反発
7. $\dot{q} = \dot{q}_{task} + \dot{q}_{null}$ にリミット近傍の方向別スケーリングを適用
8. ジョイント速度を ±1.0 rad/s にクランプ
9. **速度を積分**: $q_{target}(t+dt) = q_{target}(t) + \dot{q} \cdot dt$、ジョイントリミットでクランプ
10. $q_{target}$ と $\dot{q}$ を JointState として publish

#### 補足

- `k::nalgebra` を再エクスポートして使用 (standalone nalgebra との版競合を回避)
- URDF・メッシュファイルは `bringup` パッケージに統合済み (`bringup/urdf/sekirei.urdf`, `bringup/meshes/`)
- `urdf_path` は `control.launch.py` から `bringup` パッケージの `sekirei.urdf` パスを注入

---

### 3.3 `arm_driver` — Dynamixel アーム + グリッパ

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/hardware_drivers/arm_driver/` |
| 言語 | Rust |
| ビルド | ament_cargo |
| 依存 | rclrs, std_msgs, sensor_msgs, custom_interfaces, dynamixel2, anyhow |

#### 概要

6DOF アームおよびグリッパの Dynamixel XM サーボに対する統合ドライバ。同一の RS485 バスに接続されたアーム（ID 21-26）とグリッパ（ID 10）を一括管理し、シリアルポートの競合を回避する。`arm_backend:=direct` (デフォルト) 時に使用される。

#### トピック

| 方向 | トピック | 型 |
|------|---------|------|
| Subscribe | `/arm_joint_commands` | sensor_msgs/JointState (.position) |
| Subscribe | `/gripper_cmd` | custom_interfaces/GripperCommand |
| Subscribe | `/emergency_stop` | std_msgs/Bool (transient_local, reliable) |
| Publish | `/joint_states` | sensor_msgs/JointState (アーム角度) |
| Publish | `/gripper_status` | custom_interfaces/GripperStatus |

---

### 3.4 `crawler_driver` — Roboclaw 走行モータ制御

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/hardware_drivers/crawler_driver/` |
| 言語 | C++ |
| ビルド | ament_cmake |
| 依存 | rclcpp, geometry_msgs, std_msgs, custom_interfaces, boost::asio |

#### 概要

Roboclaw モータコントローラとシリアル通信し、左右クローラの速度を制御するドライバノード。テレオペ (`/crawler_driver`) と自律走行 (`/cmd_vel`) の両方に対応。

#### ROS パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `serial_port` | string | `"/dev/roboclaw"` | シリアルポート |
| `crawler_circumference` | double | `0.39` | クローラ周長 [m] |
| `counts_per_rev` | int | `256` | エンコーダカウント / 回転 |
| `gearhead_ratio` | int | `66` | ギアヘッド減速比 |
| `pulley_ratio` | int | `2` | プーリ比 |
| `watchdog_timeout_ms` | int | `500` | 安全停止タイムアウト [ms] |
| `track_width` | double | `0.4` | 左右トラック間距離 [m] |
| `m1_kp` | double | `0.464` | M1 モータ PID P ゲイン |
| `m1_ki` | double | `0.021` | M1 モータ PID I ゲイン |
| `m1_kd` | double | `0.0` | M1 モータ PID D ゲイン |
| `m1_qpps` | int | `53250` | M1 最大エンコーダパルス/s |
| `m2_kp` | double | `0.438` | M2 モータ PID P ゲイン |
| `m2_ki` | double | `0.020` | M2 モータ PID I ゲイン |
| `m2_kd` | double | `0.0` | M2 モータ PID D ゲイン |
| `m2_qpps` | int | `50062` | M2 最大エンコーダパルス/s |

#### トピック

| 方向 | トピック | 型 | 用途 |
|------|---------|------|------|
| Subscribe | `/crawler_driver` | CrawlerVelocity | テレオペ指令 |
| Subscribe | `/cmd_vel` | geometry_msgs/Twist | Nav2 自律走行指令 |
| Subscribe | `/emergency_stop` | std_msgs/Bool | E-Stop (transient_local) |

#### 動作詳細

- **Roboclaw シリアルプロトコル**: CRC16 チェックサム付きバイナリコマンド
- **シリアル読み取りタイムアウト**: termios `VTIME=2` (200ms) で ACK 読み取りの無限ブロックを防止
- **速度変換**: `counts/s = velocity × (counts_per_rev × gearhead × pulley) / circumference`
- **差動駆動変換** (`/cmd_vel`): `m1 = linear.x - angular.z × track_width / 2`, `m2 = linear.x + angular.z × track_width / 2`
- **PID 定数**: ROS パラメータで設定可能 (m1_kp/ki/kd/qpps, m2_kp/ki/kd/qpps)
- **ウォッチドッグ**: 500ms 無指令で全モータ停止
- **E-Stop**: `/emergency_stop` 受信時、全モータ即座停止、以降の指令を無視
- **グレースフルシャットダウン**: デストラクタ `~CrawlerDriver()` が `stopMotors()` を呼出 (Ctrl+C / クラッシュ時にモータ暴走を防止)

---

### 3.5 `flipper_driver` — Dynamixel フリッパ制御 (C++)

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/hardware_drivers/flipper_driver/` |
| 言語 | C++ |
| ビルド | ament_cmake |
| 依存 | rclcpp, std_msgs, custom_interfaces, dynamixel_workbench_toolbox |

#### 概要

Dynamixel XM シリーズを Wheel Mode (連続回転) で制御し、4 つのフリッパの速度指令を処理するドライバノード。

#### ROS パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `port_name` | string | `"/dev/dynamixel"` | シリアルポート |
| `baud_rate` | int | `115200` | ボーレート |
| `dynamixel_ids` | int[] | `[1, 2, 3, 4]` | モータ ID (左後, 右後, 左前, 右前) |
| `watchdog_timeout_ms` | int | `500` | 安全停止タイムアウト [ms] |
| `velocity_limit` | int | `1023` | Dynamixel 速度上限 (0–1023) |

#### トピック

| 方向 | トピック | 型 |
|------|---------|------|
| Subscribe | `/flipper_driver` | FlipperVelocity |
| Subscribe | `/emergency_stop` | std_msgs/Bool (transient_local, reliable) |

#### 動作詳細

- `DynamixelWorkbench` ライブラリで Wheel Mode 設定 (速度制限は `velocity_limit` パラメータ)
- 初期化時 `wheelMode()` / `itemWrite()` の戻り値をチェック、失敗時はノード停止
- `flipper_vel` 配列のインデックスが Dynamixel ID に対応
- `stopMotors()` の `goalVelocity()` 戻り値をチェック、失敗時はエラーログ出力
- ウォッチドッグで無指令時に全モータ停止
- **E-Stop**: `/emergency_stop` 受信時、全モータ即座停止、以降の指令を無視
- **グレースフルシャットダウン**: デストラクタ `~FlipperDriver()` が `stopMotors()` を呼出

---

### 3.6 `sensor_gateway` — STM32 IMU シリアルブリッジ

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/hardware_drivers/sensor_gateway/` |
| 言語 | Rust |
| ビルド | ament_cargo |
| 依存 | rclrs, sensor_msgs, std_msgs, builtin_interfaces, anyhow |
| ソース | `main.rs` (ノード), `imu.rs` (データ構造・変換) |

#### 概要

STM32 マイコンが BNO055 IMU から取得した姿勢データを UART で受信し、ROS 2 の `sensor_msgs/Imu` として配信するブリッジノード。

#### ROS パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `serial_port` | string | `"/dev/stm32"` | STM32 シリアルデバイス |
| `baud_rate` | int | `115200` | ボーレート |

#### トピック

| 方向 | トピック | 型 | 頻度 |
|------|---------|------|------|
| Publish | `/imu/data` | sensor_msgs/Imu | ~50 Hz |
| Publish | `/imu/health` | std_msgs/Bool | 1 Hz |

`/imu/health`: 直近 2 秒以内にシリアルデータを受信していれば `true`、それ以外 (デバイス未接続・切断中・リトライ中) は `false`。`odom_selector` がこれを購読してオドメトリソースを自動切り替えする。

#### 動作詳細

- **シリアル受信スレッド**: CSV 形式 `heading,roll,pitch,sys_cal,gyr_cal,acc_cal,mag_cal` を行単位で読み取り
- **Euler → Quaternion 変換**: ZYX Intrinsic 回転で $(w, x, y, z)$ を計算
- **共分散行列**:
  - Orientation: 対角 0.01 (較正済み推定値)
  - Angular velocity / Linear acceleration: 不明 (先頭 -1.0, REP-145 準拠)
- **タイマー**: ~50Hz で配信、新データがある場合のみ (freshness flag)
- **キャリブレーション**: 50 メッセージごとにログ出力
- **自動再接続**: シリアルエラー時に 2 秒待って再試行
- **ユニットテスト**: CSV パース・四元数変換のテスト付き
- **IMU 死活監視**: `last_recv` タイムスタンプを共有 Mutex で管理し、1Hz タイマーで健全性を `/imu/health` に配信

---

### 3.7 `audio_bridge` — GStreamer/Opus 双方向音声通信

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/audio_bridge/` |
| 言語 | Python |
| ビルド | ament_python |
| 依存 | rclpy, custom_interfaces, python3-gi (GStreamer) |

#### 概要

GStreamer + Opus コーデックを使用してロボット↔オペレータ間で双方向音声通信を行うパッケージ。WebRTC/シグナリングサーバーは不要。Zenoh が ROS 2 トピックのトランスポートを担うため、LAN 内であれば両端とも ROS 2 ノードのみで完結する。

#### ノード構成

| ノード | ファイル | 役割 |
|--------|---------|------|
| `audio_sender_node` | `audio_sender_node.py` | マイク → Opus エンコード → ROS 2 トピック publish |
| `audio_receiver_node` | `audio_receiver_node.py` | ROS 2 トピック subscribe → Opus デコード → スピーカー |

#### 起動

`system.launch.py` の `audio.launch.py` 経由で起動 (`use_audio:=false` で無効化可能)。

---

### 3.8 `qr_detector` — QR コード検出

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/perception/qr_detector/` |
| 言語 | Python |
| ビルド | ament_python |
| 依存 | rclpy, sensor_msgs, std_msgs, cv_bridge |

#### 概要

OpenCV の WeChatQRCode デテクタを使い、カメラ画像から QR コードを検出・デコードするノード。検出結果のテキストと、バウンディングボックス付きの圧縮画像をパブリッシュする。

#### ROS パラメータ

| パラメータ | 型 | デフォルト | 説明 |
|-----------|------|-----------|------|
| `model_dir` | string | auto | WeChatQRCode モデルディレクトリ |
| `publish_compressed` | bool | `true` | 注釈付き JPEG を配信するか |
| `jpeg_quality` | int | `60` | JPEG 圧縮品質 |
| `detection_interval` | int | `1` | N フレームに 1 回検出 |

#### トピック

| 方向 | トピック | 型 | QoS |
|------|---------|------|-----|
| Subscribe | `/camera/image_raw` | sensor_msgs/Image | BEST_EFFORT |
| Publish | `/qr_codes` | std_msgs/String | — |
| Publish | `/image/compressed` | sensor_msgs/CompressedImage | BEST_EFFORT (sensor_qos) |

#### 動作詳細

- Caffe モデル 4 ファイル (`models/` ディレクトリ) を使用
- `detection_interval` でフレームスキップ (計算負荷低減)
- 検出した QR コードにバウンディングポリゴンとテキストを描画
- JPEG 圧縮した注釈付き画像をリモート監視用に配信

---

### 3.9 `qr_detector_cpp` — QR コード検出 (C++ / Component)

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/perception/qr_detector_cpp/` |
| 言語 | C++ |
| ビルド | ament_cmake |
| 依存 | rclcpp, rclcpp_components, sensor_msgs, std_msgs, cv_bridge, ament_index_cpp |

#### 概要

`qr_detector` (Python) の C++ 移植版。`rclcpp_components` として実装されており、`v4l2_camera` と同一プロセス内（ComposableNodeContainer）で動作させることで、画像転送のオーバーヘッドをゼロに抑えている。

#### ROS パラメータ / トピック

- **Subscribe**: `/camera/image_raw` (sensor_msgs/Image) — **Intra-process 通信推奨**
- **Publish**: `/qr_codes` (std_msgs/String)

---

### 3.10 `spark_fast_lio` — LiDAR 慣性オドメトリ

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/external/spark-fast-lio/` (Git サブモジュール) |
| 言語 | C++ |
| ビルド | ament_cmake + rclcpp_components |

#### 概要

FAST-LIO2 ベースの LiDAR-IMU 融合オドメトリ。VLP-16 のポイントクラウドと BNO055 の IMU データからリアルタイムでロボットの姿勢を推定する。

#### トピック

| 方向 | トピック | 型 |
|------|---------|------|
| Subscribe | `/velodyne_points` (remapped `lidar`) | sensor_msgs/PointCloud2 |
| Subscribe | `/imu/data` (remapped `imu`) | sensor_msgs/Imu |
| Publish | TF: `odom` → `base_link` | tf2_msgs/TFMessage |

> **Note**: 以前は `fix_pointcloud_time` ノードで `/velodyne_points_fixed` に変換してから入力していたが、`timestamp_unit: 0` (SEC) に変更したことで VLP-16 の `time` フィールドを直接読むようにした。Python ノードの全点ループが不要になり、レイテンシと CPU 負荷が大幅に改善。

#### 設定 (`spark_fast_lio_min.yaml`)

| パラメータ | 値 | 説明 |
|-----------|------|------|
| `lidar_type` | 2 (VLP-16) | LiDAR 種別 |
| `scan_line` | 16 | スキャンライン数 |
| `scan_rate` | 15 | スキャンレート [Hz] |
| `timestamp_unit` | 0 (SEC) | ポイント time フィールドの単位 (0=秒, 1=ms, 2=μs, 3=ns) |
| `fov_degree` | 360 | 視野角 |
| `det_range` | 100.0 | 最大検出距離 [m] |
| `blind` | 0.5 | 近距離除外 [m] |
| `filter_size_map` | 0.5 | ダウンサンプリングサイズ [m] |
| `acc_cov` / `gyr_cov` | 0.1 | IMU ノイズ共分散 |

---

### 3.11 `bringup` — Launch / Config / URDF / Scripts

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/bringup/` |
| 言語 | CMake + Python + URDF/Xacro |
| ビルド | ament_cmake |
| 依存 | rclpy, sensor_msgs, xacro, robot_state_publisher, ros_gz_sim, ros_gz_bridge, foxglove_bridge |

#### 概要

全 Launch ファイル、設定 YAML、URDF モデル、ユーティリティスクリプトを含む統合パッケージ。アーム IK 用の URDF (`sekirei.urdf`) とメッシュ (STL 11 個) もここに統合されている。

#### Launch ファイル

| ファイル | 目的 |
|----------|------|
| `system.launch.py` | マスター (NUC): robot_state_publisher → network → audio → perception → camera → control → nav2 を順に起動。foxglove_bridge はオペレータ側で起動するためここには含まない |
| `network.launch.py` | `RMW_IMPLEMENTATION=rmw_zenoh_cpp` + `RMW_ZENOH_CONFIG_URI=file://.../zenoh_robot.json5` を ROS 2 ノードへ設定し、`zenohd --config zenoh_router.json5` を起動 (QUIC/7447, auto-respawn) |
| `control.launch.py` | ドライバ (crawler, flipper, sensor_gateway, arm_gripper) + コントローラ (joy_controller, arm_controller)。`arm_backend` 引数でアームバックエンドを選択 |
| `perception.launch.py` | spark_fast_lio, velodyne (optional), pointcloud_to_laserscan, time fix, dummy IMU, slam_toolbox, rviz |
| `nav2.launch.py` | Nav2: controller_server, planner_server, behavior_server, bt_navigator, velocity_smoother, lifecycle_manager |
| `camera.launch.py` | USB カメラ (v4l2_camera + ffmpeg_image_transport H.264) + qr_detector_cpp (intra-process composable) |
| `display.launch.py` | ロボットモデル表示 (robot_state_publisher + joint_state_publisher_gui + RViz2) |
| `simulation.launch.py` | Gazebo Harmonic + robot_state_publisher + spawn + ros_gz_bridge + pointcloud_to_laserscan + odom_tf_bridge + slam_toolbox + lifecycle_manager + RViz |

**オペレータ側 Launch ファイル** (`operator_ws/launch/`):

| ファイル | 目的 |
|----------|------|
| `operator.launch.py` | オペレータ PC: `RMW_ZENOH_CONFIG_URI=zenoh_ope.json5` + `joy_node` + `foxglove_bridge (:8765)` |

#### RViz Config

| ファイル | 目的 |
|----------|------|
| `rviz/simulation.rviz` | シミュレーション用 RViz 設定 (Map + GlobalCostmap + LocalCostmap + Path + LaserScan + PointCloud2 + RobotModel + TF + Odometry + Camera, fixed frame: map, Nav2 GoalTool 付き) |
| `rviz/display.rviz` | `display.launch.py` 用 RViz 設定 (Grid + RobotModel + TF, fixed frame: base_link) |

#### スクリプト

| ファイル | 目的 |
|----------|------|
| `crawler_vel_bridge.py` | CrawlerVelocity (m1, m2) → Twist (cmd_vel) 変換。`linear.x = (m1+m2)/2`, `angular.z = (m2-m1)/track_width` |
| `odom_tf_bridge.py` | Gazebo の `/odom` (Odometry) → `odom→base_footprint` TF 変換。Gazebo DiffDrive が TF を直接出さないため必要 |
| `arm_gz_bridge.py` | `/arm_joint_commands` (JointState) → Gazebo の per-joint `/model/.../cmd_pos` (Float64) 変換 |
| `arm_cmd_bridge.py` | `/arm_joint_commands` (JointState) → `/forward_position_controller/commands` (Float64MultiArray) 変換。`arm_backend:=ros2_control` 時のみ起動 |
| `dummy_imu_node.py` | 単位姿勢 + 重力加速度の IMU を 100Hz で配信 (テスト用) |
| `fix_pointcloud_time_node.py` | VLP-16 のポイント毎タイムスタンプを方位角から再計算。`timestamp_unit: 0` 対応によりデフォルト無効、必要時は `use_time_fix:=true` |
| `odom_selector.py` | `/imu/health` に基づいて `/odom` ソースを切り替え。IMU 正常時は spark_fast_lio、IMU 死亡時は KISS-ICP へフォールバック (ヒステリシスあり: 死亡判定 1s、復活判定 3s) |
| `demo_joints.py` | フリッパ・アーム全軸をサイン波でアニメーションする `/joint_states` を 20Hz で配信 (モデル確認用) |
| `teleop_joints.py` | キーボードでジョイントを直接操作する仮想テレオペ (開発用) |
| `ffmpeg_to_foxglove_video.py` | `FFMPEGPacket` → `foxglove_msgs/CompressedVideo` 変換。Foxglove Studio でネイティブ H.264 表示するために必要 |

#### URDF

| ファイル | 用途 |
|----------|------|
| `urdf/robot.urdf.xacro` (~640 行) | シミュレーション用の完全なロボットモデル (Gazebo プラグイン含む) |
| `urdf/sekirei.urdf` | アーム IK 用 URDF (`arm_controller` が参照、`k` クレートでロード) |

##### `robot.urdf.xacro` の構成

シミュレーション用の完全なロボットモデル:

| コンポーネント | 仕様 |
|---------------|------|
| **ベース** | 0.60×0.35×0.15m, 15kg, 差動2輪 + 後部キャスター |
| **フリッパ (×4)** | FR, FL, BR, BL, revolute ジョイント, STL メッシュ |
| **6DOF アーム** | joint1 (Yaw) → joint2 (Shoulder) → joint3 (Elbow) → joint4-6 (Wrist) → link_tip |
| **VLP-16 LiDAR** | lidar, 360×16 サンプル, 8Hz, `/velodyne_points` |
| **IMU** | 100Hz, ガウシアンノイズ |
| **カメラ** | 640×480, 30fps |

Gazebo Harmonic プラグイン:

| プラグイン | 機能 |
|-----------|------|
| `gz-sim-diff-drive-system` | 差動駆動 (cmd_vel → odom) |
| `gz-sim-joint-state-publisher-system` | 10 ジョイントの状態配信 |
| `gz-sim-joint-position-controller-system` | フリッパ (×4) + アーム (×6) の PID 位置制御 |
| `gz-sim-sensors-system` | LiDAR / IMU / カメラ (world 側で有効化, `render_engine=ogre`) |

> **Note**: `rescue_field.sdf` は `gz-sim-sensors-system` を `render_engine=ogre` で有効化している。GUI や OpenGL 周りが不安定な環境では `simulation.launch.py headless:=true` または `use_gz_gui:=false` を使って起動し、`/velodyne_points` → `/scan` の ROS 側変換は維持する。

---

### 3.12 `custom_interfaces` — カスタムメッセージ定義

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/custom_interfaces/` |
| 言語 | CMake (rosidl コード生成) |
| ビルド | ament_cmake + rosidl_default_generators |

#### メッセージ

##### `CrawlerVelocity.msg`

```
float32 m1_vel    # 左クローラ速度 [m/s]
float32 m2_vel    # 右クローラ速度 [m/s]
```

##### `FlipperVelocity.msg`

```
int32[] flipper_vel   # [左後, 右後, 左前, 右前] エンコーダ単位
```

##### `GripperCommand.msg`

```
uint16 position       # 目標位置 [Dynamixel 単位]
uint16 max_current    # 最大電流制限 [mA]
```

##### `GripperStatus.msg`

```
uint8 IDLE=0
uint8 MOVING=1
uint8 GRIPPING=2
uint8 OVERLOAD=3
uint8 ERROR=4

uint8  state          # 状態定数
int32  position       # 現在位置
int16  current        # 現在電流 [mA]
uint8  temperature    # 温度 [°C]
```

---

### 3.13 `stm32_ws` — STM32 ファームウェア

| 項目 | 内容 |
|------|------|
| パス | `stm32_ws/` |
| 言語 | Rust (ベアメタル) |
| ターゲット | `thumbv7em-none-eabihf` (STM32F446) |
| 依存 | cortex-m, cortex-m-rt, panic-halt, embedded-hal, stm32f4xx-hal |

#### 概要

STM32F446 上で BNO055 IMU を I2C で読み取り、CSV 形式で UART 出力するベアメタルファームウェア。`sensor_gateway` ノードがこの UART データを受信する。

---

### 3.14 補助パッケージ群

#### `omron_bridge` — OMRON 2JCIE-BU01 環境センサブリッジ

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/perception/omron_bridge/` |
| 言語 | Python |
| ビルド | ament_python |
| 依存 | rclpy, std_msgs, sensor_msgs |

OMRON 2JCIE-BU01 (USB 型環境センサ) から温度・湿度・気圧・CO2 等を取得し、ROS 2 トピックとして配信する。

#### `robo_map_exporter` — SLAM マップエクスポータ

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/perception/robo_map_exporter/` |
| 言語 | Python |
| ビルド | ament_cmake (ament_python) |
| 依存 | rclpy, nav_msgs, sensor_msgs, geometry_msgs, std_srvs, tf2_ros |

slam_toolbox が生成した OccupancyGrid または PointCloud2 を PLY / CSV 形式でファイル出力する PoC パッケージ。外部 3D ツールへのマップ持ち出し用。

#### `h264_republisher` — H.264 デコードブリッジ

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/extras/h264_republisher/` |
| 言語 | C++ |
| ビルド | ament_cmake |
| 依存 | rclcpp, sensor_msgs, cv_bridge, image_transport, opencv4 |

ffmpeg_image_transport の `FFMPEGPacket` トピックを FFmpeg/libav でデコードし、`sensor_msgs/Image` として再配信するデバッグ用ノード。Foxglove ではなく RViz でカメラ映像を確認する場合に使用する。

#### `vision_processor` — OpenCV QR 検出 (Rust / 実験的)

| 項目 | 内容 |
|------|------|
| パス | `main_ws/src/perception/vision_processor/` |
| 言語 | Rust |
| ビルド | ament_cargo |
| 依存 | rclrs, opencv (WeChatQRCode) |

Rust バインディングから OpenCV WeChatQRCode を呼び出す実験的な QR 検出実装。本運用では `qr_detector_cpp` を使用しており、このパッケージは現在ビルド対象外。

---

## 4. 通信アーキテクチャ

### 4.1 Zenoh ミドルウェア

ロボットとオペレータ間の ROS 2 通信は `rmw_zenoh_cpp` を使います。

| コンポーネント | 設定ファイル | mode | 接続 |
|-------------|------------|------|------|
| `zenohd` (NUC デーモン) | `zenoh_router.json5` | `router` | listen quic/0.0.0.0:7447 |
| ロボット ROS2 ノード (NUC) | `zenoh_robot.json5` | `client` | connect quic/127.0.0.1:7447 (→ローカル zenohd) |
| オペレータ ROS2 ノード | `zenoh_ope.json5` | `client` | connect quic/<ROBOT_IP>:7447 (→リモート zenohd) |

`zenohd` はロボット上で単一のルーターデーモンとして動作します。
ROS2 ノード自身が router になることはありません（ポート競合防止のため）。
オペレータ側は `zenohd` なし・環境変数設定のみで動作します。

### 4.2 映像配信 (H.264)

Wi-Fi 帯域を節約しつつ低遅延な映像配信を実現するため、以下のスタックを使用します。

1. **ドライバ**: `v4l2_camera` (composable node, intra-process 通信)
2. **エンコード**: `ffmpeg_image_transport` (H.264 / libx264)
3. **設定**: `profile=baseline`, `preset=ultrafast`, `tune=zerolatency`, `bframes=0`
4. **トピック**: `/camera/image_raw/ffmpeg` (FFMPEGPacket 型)
5. **Foxglove 表示**: `ffmpeg_to_foxglove_video` ノードで `foxglove_msgs/CompressedVideo` に変換後、Foxglove Studio がネイティブデコード

### 4.3 オペレータ GUI

- **Foxglove Bridge**: オペレータ PC 側で `operator.launch.py` から起動し、Zenoh 経由で受信したデータを Foxglove Studio へ供給する。
- **アドレス**: `ws://127.0.0.1:8765` (Foxglove Studio の接続先)
- **映像**: `/camera/image_raw/ffmpeg` → `foxglove_msgs/CompressedVideo` (H.264, 2Mbps デフォルト)

### 4.4 アームバックエンド選択 (`arm_backend` 引数)

`control.launch.py` の `arm_backend` 引数でアームドライバを切り替えられます。

| 値 | バックエンド | 説明 |
|----|-------------|------|
| `direct` (デフォルト) | `arm_driver` (Rust) | Dynamixel を直接制御。シンプルで低レイテンシ |
| `ros2_control` | `dynamixel_hardware_interface` + `controller_manager` | ros2_control エコシステムに統合。MoveIt2 / joint_trajectory_controller が利用可能 |

`ros2_control` 使用時は `arm_cmd_bridge` ノードが `/arm_joint_commands` → `/forward_position_controller/commands` 変換を担う。MoveIt2 を使う場合は起動後に:

```bash
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate   joint_trajectory_controller
```

---

## 5. トピック構成

### 5.1 制御系トピック

| トピック | 型 | Publisher | Subscriber |
|----------|------|-----------|------------|
| `/joy` | sensor_msgs/Joy | joy_node | joy_controller |
| `/crawler_driver` | CrawlerVelocity | joy_controller | crawler_driver |
| `/flipper_driver` | FlipperVelocity | joy_controller | flipper_driver |
| `/arm_cmd_vel` | geometry_msgs/Twist | joy_controller | arm_controller |
| `/arm_joint_commands` | sensor_msgs/JointState | arm_controller | arm_driver / arm_cmd_bridge / arm_gz_bridge |
| `/joint_states` | sensor_msgs/JointState | arm_driver / flipper_driver / Gazebo | arm_controller, (RViz) |
| `/gripper_cmd` | GripperCommand | (operator) | arm_gripper_driver |
| `/gripper_status` | GripperStatus | arm_gripper_driver | (operator) |
| `/cmd_vel` | geometry_msgs/Twist | Nav2 / ros_gz_bridge | crawler_driver / Gazebo |
| `/emergency_stop` | std_msgs/Bool | joy_controller (PSボタン) | arm_gripper_driver, crawler_driver, flipper_driver, arm_controller |

> **E-Stop QoS**: transient_local + reliable (depth 1)。過去に publish された latched メッセージも新規 subscriber が受信する。
> **E-Stop リセット**: 現在は未実装。全ノード再起動が必要。

### 5.2 認識系トピック

| トピック | 型 | Publisher | Subscriber |
|----------|------|-----------|------------|
| `/velodyne_points` | PointCloud2 | velodyne / Gazebo | spark_fast_lio, pointcloud_to_laserscan |
| `/velodyne_points_fixed` | PointCloud2 | fix_pointcloud_time (`use_time_fix:=true` 時のみ) | spark_fast_lio (`use_time_fix:=true` 時のみ) |
| `/imu/data` | sensor_msgs/Imu | sensor_gateway / Gazebo | spark_fast_lio |
| `/imu/health` | std_msgs/Bool | sensor_gateway | odom_selector |
| `/scan` | LaserScan | pointcloud_to_laserscan | slam_toolbox, Nav2 |
| `/map` | OccupancyGrid | slam_toolbox | Nav2 |
| `/odom` | Odometry | spark_fast_lio / odom_selector / Gazebo | Nav2 (bt_navigator, velocity_smoother) |

> **Note**: spark_fast_lio はデフォルトで `/odometry` に publish するが、`perception.launch.py` で `/odom` に remap している。

> **Note**: `fix_pointcloud_time` ノードは `use_time_fix:=true` を明示しない限り起動しない。deskew に問題がある場合のみ有効化する。

> **Note**: `odom_selector` は `/imu/health` が 1 秒間 false になると自動で KISS-ICP の `/kiss/odometry` にフォールバックし、3 秒間 true が続くと spark_fast_lio に戻る。

### 5.3 カメラ系トピック

| トピック | 型 | Publisher | Subscriber |
|----------|------|-----------|------------|
| `/camera/image_raw` | Image | v4l2_camera / Gazebo | qr_detector_cpp (intra-process) |
| `/camera/image_raw/ffmpeg` | FFMPEGPacket (H.264) | v4l2_camera (ffmpeg_image_transport) | ffmpeg_to_foxglove_video |
| `/camera/image_raw/foxglove_video` | foxglove_msgs/CompressedVideo | ffmpeg_to_foxglove_video | foxglove_bridge (operator) |
| `/camera/camera_info` | CameraInfo | v4l2_camera / Gazebo | — |
| `/camera/qr_codes` | std_msgs/String | qr_detector_cpp | (operator) |

### 5.4 シミュレーション固有

| トピック | 型 | Publisher | Subscriber |
|----------|------|-----------|------------|
| `/clock` | rosgraph_msgs/Clock | Gazebo (ros_gz_bridge) | 全ノード (use_sim_time) |

---

## 6. TF ツリー

### 実機

```
map                ← slam_toolbox
 └─ odom           ← spark_fast_lio (または odom_selector 経由 KISS-ICP)
     └─ base_link
         ├─ velodyne       ← static_transform_publisher
         ├─ imu_link       ← URDF (fixed)
         ├─ camera_link    ← URDF (fixed)
         ├─ flipper_*_link ← URDF (revolute)
         └─ arm_base_link
             └─ arm_link1 → ... → link_tip
```

### シミュレーション

```
map                ← slam_toolbox (lifecycle_manager_slam でアクティベート)
 └─ odom           ← odom_tf_bridge (/odom Odometry → TF 変換)
     └─ base_footprint
         └─ base_link  ← robot_state_publisher (URDF)
             ├─ left_wheel / right_wheel
             ├─ velodyne_link
             ├─ imu_link
             ├─ camera_link
             ├─ flipper_*_link (×4)
             └─ arm_base_link → arm_link1-6 → link_tip
```

> **Note**: Jazzy の `slam_toolbox` は Lifecycle ノードのため、`lifecycle_manager_slam` (autostart=True) で
> 明示的にアクティベートしないと `/scan` を購読せず `map→odom` TF も配信されない。
> シミュレーションでは `/velodyne_points` を `pointcloud_to_laserscan` で `/scan` に変換してから SLAM に渡す。

### フレーム役割

| フレーム | 説明 |
|----------|------|
| `map` | SLAM 地図座標系 (グローバル, ドリフト補正済み) |
| `odom` | オドメトリ座標系 (連続だがドリフトする) |
| `base_footprint` | 地面投影点 (シミュレーション) |
| `base_link` | ロボット本体中心 |
| `velodyne` / `velodyne_link` | VLP-16 LiDAR 取付位置 |
| `link_tip` | アーム先端 (エンドエフェクタ) |

---

## 7. Launch 構成

### ロボット側 (NUC)

```
system.launch.py               … NUC 側マスター Launch (foxglove_bridge は含まない)
 ├─ robot_state_publisher     … URDF → TF (base_link→各センサ/アーム/フリッパ)
 ├─ network.launch.py        … Zenoh (RMW_ZENOH_CONFIG_URI=zenoh_robot.json5 + zenohd --config zenoh_router.json5, QUIC/7447)
 ├─ audio.launch.py          … GStreamer/Opus 双方向音声 (use_audio=false で無効化可)
 ├─ perception.launch.py     … LiDAR + SLAM
 │   ├─ velodyne_driver       (use_velodyne=true 時)
 │   ├─ velodyne_pointcloud   (use_velodyne=true 時)
 │   ├─ fix_pointcloud_time   (use_time_fix=true 時, デフォルト無効)
 │   ├─ spark_fast_lio
 │   ├─ odom_selector         … /imu/health → spark_fast_lio / KISS-ICP 自動切替
 │   ├─ static_transform_publisher (base_link→velodyne)
 │   ├─ pointcloud_to_laserscan (use_scan=true 時, /velodyne_points → /scan)
 │   ├─ slam_toolbox           (use_slam=true 時)
 │   ├─ dummy_imu              (use_dummy_imu=true 時)
 │   └─ rviz2                  (use_rviz=true 時)
 ├─ camera.launch.py         … v4l2_camera (H.264) + qr_detector_cpp (composable, intra-process)
 ├─ control.launch.py        … ドライバ + コントローラ
 │   ├─ crawler_driver
 │   ├─ flipper_driver
 │   ├─ sensor_gateway
 │   ├─ [arm_backend=direct]      arm_driver (Rust 直接制御, デフォルト)
 │   ├─ [arm_backend=ros2_control] dynamixel_hardware_interface + controller_manager
 │   │                             + arm_cmd_bridge (JointState → Float64MultiArray)
 │   ├─ joy_controller        ← /joy を subscribe (オペレータから Zenoh 経由)
 │   └─ arm_controller        (urdf_path を bringup から注入)
 └─ nav2.launch.py           … 自律走行 (use_nav2=true 時)
     ├─ controller_server     (RegulatedPurePursuit)
     ├─ planner_server        (SmacPlanner2D)
     ├─ behavior_server
     ├─ bt_navigator
     ├─ velocity_smoother
     └─ lifecycle_manager
```

### オペレータ側 (ラップトップ)

```
operator.launch.py (operator_ws/launch/)
 ├─ [env] RMW_IMPLEMENTATION=rmw_zenoh_cpp
 ├─ [env] RMW_ZENOH_CONFIG_URI=file://../zenoh_ope.json5
 ├─ joy_node                  … PS4 → /joy (Zenoh 経由で NUC の joy_controller に届く)
 └─ foxglove_bridge           … WebSocket :8765 → Foxglove Studio
```

### シミュレーション・表示用 (開発)

```
simulation.launch.py          … シミュレーション単体 (headless / GUI 切替あり)
 ├─ gz sim (Gazebo Harmonic, rescue_field.sdf / rescue_field_headless.sdf)
 ├─ robot_state_publisher
 ├─ create (spawn entity)
 ├─ parameter_bridge (ros_gz_bridge)  … /clock, /cmd_vel, /velodyne_points, /imu/data, /odom, /joint_states
 ├─ pointcloud_to_laserscan     … /velodyne_points → /scan
 ├─ odom_tf_bridge              ← /odom (Odometry) → odom→base_footprint TF
 ├─ slam_toolbox                (use_slam=true 時)
 ├─ lifecycle_manager_slam      (use_slam=true 時) ← SLAM をアクティベート (Jazzy では Lifecycle ノード)
 ├─ nav2.launch.py              (use_nav2=true 時, TimerAction 30秒遅延)
 └─ rviz2                       (use_rviz=true 時, simulation.rviz)

display.launch.py             … ロボットモデル表示
 ├─ robot_state_publisher     (URDF → /robot_description, TF)
 ├─ joint_state_publisher_gui (スライダーでジョイント操作)
 └─ rviz2                     (use_rviz=true 時, display.rviz)
```

---

## 8. 設定ファイル一覧

`bringup/config/` 内の全設定ファイル:

| ファイル | 対象 | 主要パラメータ |
|----------|------|----------------|
| `crawler_driver.yaml` | crawler_driver | serial_port, circumference, counts/rev, gearhead, pulley, track_width |
| `flipper_driver.yaml` | flipper_driver | port_name, baud_rate, dynamixel_ids [1,2,3,4], watchdog |
| `arm_controller.yaml` | arm_controller | urdf_path, end_link, watchdog_timeout_ms |
| `arm_gripper_driver.yaml` | arm_driver | port_name, baud_rate, arm_ids, gripper_id, gripper_max_current |
| `ros2_control_controllers.yaml` | controller_manager | forward_position_controller, joint_trajectory_controller の設定 |
| `sensor_gateway.yaml` | sensor_gateway | serial_port (/dev/stm32), baud_rate (115200) |
| `joy_controller.yaml` | joy_controller | max_speed, deadzone, flipper_speed, arm_linear_scale, arm_angular_scale |
| `usb_cam_params.yaml` | v4l2_camera | video_device, image_size, pixel_format |
| `nav2_params.yaml` | Nav2 全体 | RegulatedPurePursuit (0.3 m/s), SmacPlanner2D, footprint 0.60×0.35m 矩形, inflation 0.55m |
| `dummy_imu.yaml` | dummy_imu_node | 単位姿勢 + 重力加速度の IMU を 100Hz で配信 (テスト用) |
| `spark_fast_lio_min.yaml` | spark_fast_lio | VLP-16 設定, timestamp_unit: 0 (SEC), map_frame: odom, filter_size: 0.5m |
| `fix_pointcloud_time.yaml` | fix_pointcloud_time | input/output_topic (デフォルト無効, use_time_fix:=true 時に使用) |
| `pointcloud_to_laserscan.yaml` | pointcloud_to_laserscan | target_frame: base_link, height: -0.2~0.2m |
| `slam_toolbox_async.yaml` | slam_toolbox | Ceres solver, scan_topic: /scan |
| `velodyne_driver.yaml` | velodyne_driver | IP: 10.42.0.242, port: 2368, VLP16, 900 RPM |
| `velodyne_pointcloud.yaml` | velodyne_pointcloud | organized cloud, range: 0.9–130m |
| `zenoh_robot.json5` | ロボット側 ROS 2 ノード | mode=client, connect quic/127.0.0.1:7447 |
| `zenoh_router.json5` | zenohd | mode=router, listen quic/0.0.0.0:7447, SHM 有効, no multicast |

`bringup/config/` 外の Zenoh 設定:

| ファイル | 場所 | 主要パラメータ |
|----------|------|----------------|
| `zenoh_ope.json5` | `operator_ws/` | mode=client, connect quic/\<ROBOT_IP\>:7447, no multicast |

---

## 9. データフロー図

```mermaid
graph LR
  subgraph Operator["Operator PC"]
    PS4[PS4 Controller] --> joy_node
    foxglove_bridge[foxglove_bridge<br>:8765] -->|WebSocket| Foxglove[Foxglove Studio]
  end

  joy_node -->|/joy<br>Zenoh QUIC| joy_controller

  subgraph NUC["NUC ロボット側"]
    joy_controller -->|/crawler_driver| crawler_driver
    joy_controller -->|/flipper_driver| flipper_driver
    joy_controller -->|/arm_cmd_vel| arm_controller
    arm_controller -->|/arm_joint_commands| arm_driver[arm_driver]
    arm_driver -->|/joint_states| arm_controller
    robot_state_publisher -->|/tf, /tf_static| tf_tree
  end

  subgraph Hardware
    crawler_driver --> Roboclaw
    flipper_driver --> Dynamixel_F[Dynamixel Flippers]
    arm_driver --> Dynamixel_AG[Dynamixel Arm + Gripper]
    sensor_gateway --> STM32[STM32 + BNO055]
  end

  subgraph Perception
    VLP16[VLP-16 LiDAR / Gazebo] -->|/velodyne_points| ros_gz_bridge
    ros_gz_bridge -->|/velodyne_points| pointcloud_to_laserscan[p2l]
    pointcloud_to_laserscan -->|/scan| slam_toolbox
    STM32 --> sensor_gateway -->|/imu/data| spark_fast_lio
    sensor_gateway -->|/imu/health| odom_selector
    spark_fast_lio -->|/spark_lio/odom| odom_selector
    kiss_icp[kiss-icp] -->|/kiss/odometry| odom_selector
    odom_selector -->|/odom| Nav2
  end

  subgraph Navigation
    slam_toolbox -->|/map| Nav2
    Nav2 -->|/cmd_vel| crawler_driver
  end

  subgraph Camera
    USB_CAM[USB Camera] -->|/camera/image_raw| v4l2[v4l2_camera]
    v4l2 -->|/camera/image_raw intra-process| qr_detector[qr_detector_cpp]
    v4l2 -->|/camera/image_raw/ffmpeg FFMPEGPacket| ffmpeg2fox[ffmpeg_to_foxglove_video]
    ffmpeg2fox -->|foxglove_msgs/CompressedVideo| foxglove_bridge
    qr_detector -->|/camera/qr_codes| foxglove_bridge
  end

  subgraph TF
    spark_fast_lio -->|odom→base_link| tf_tree[TF Tree]
    slam_toolbox -->|map→odom| tf_tree
  end
```

詳細なトピック接続図は `topology/system_topology.mmd` を参照。

---

## 10. 外部サブモジュール

`main_ws/src/external/` の Git サブモジュール:

| サブモジュール | 用途 |
|---------------|------|
| `ros2_rust/` | rclrs — Rust 用 ROS 2 クライアントライブラリ |
| `rosidl_rust/` | rosidl_generator_rs — Rust メッセージ生成器 |
| `spark-fast-lio/` | FAST-LIO2 ベース LiDAR 慣性オドメトリ |
| `kiss-icp/` | ICP ベース LiDAR オドメトリ。IMU 障害時の `odom_selector` フォールバック先 (ビルド対象外は設定次第) |
| `dynamixel_hardware_interface/` | ros2_control 対応 Dynamixel ハードウェアインターフェース。`arm_backend:=ros2_control` 時に使用 |
| `common_interfaces/` | 標準 ROS 2 メッセージ (std_msgs, sensor_msgs 等) |
| `rcl_interfaces/` | ROS 2 パラメータ・サービスインターフェース |
| `rosidl_defaults/` | rosidl デフォルト生成器 / ランタイム |
| `unique_identifier_msgs/` | UUID メッセージ |
| `test_interface_files/` | テスト用メッセージ定義 |
