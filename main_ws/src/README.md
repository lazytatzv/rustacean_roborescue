
# 制御外観

## CONCEPT

低レイヤーはRustの積極採用、SLAMやEKF周りは枯れているC++ライブラリを採用
Visionは実用性でPython


## 入力系(Operator Interface)

### joy_node (C++既存)

ps4コントローラの入力を配信

- pub topic: 
  - /joy (sensor_msgs/Joy)

### joy_operator (Rust)

人間のJoystickからの操作をロボットへの物理指令へと変換する層

- sub topic: 
  - /joy (sensor_msgs/Joy)

- pub topic: 
  - /flipper_cmd_vel (std_msgs/Float64MultiArray)
    - 4つのフリッパーの目標速度を配列で送るのがシンプル

  - /crawler_cmd_vel (custom)
    - crawler制御
    - 直感的なのでTwistをあえて採用しない
    - [left_vel, right_vel]

  - /arm_cmd_vel (geometry_msgs/Twist)
    - アーム制御. End Effectorをどれくらいの速度で動かすか

## Mobility

### crawler_driver (rust)

速度入力をcustom msgから受付け、実際にMaxonを駆動する
PIDとかもやる
ros2_controllerは安定しなさそうなので多分つかわない

FFをいれるとレスポンスがよくなるかも
Torque/Current Limitは絶対にいれる(Software Fuse)！

- sub topic:
  - /cmd_vel (geometry_msgs/Twist)
    - 自律移動とかナビゲーション用
    - ドライバ内でIKを解いて、左右の目標速度にする

  - /crawler_cmd_vel (custom)
    - 人間の操作用(優先度高め)

- pub topic: 
  - /odom_wheel (nav_msgs/Odometry)
    - encoderの精度がたとえ悪くても絶対に吐いておく
  - /crawler_status (sensor_msgs/JointState)
    - モータの温度、電流値などをのせたい



### flipper_driver (rust)

Dynamixel制御

- sub topic:
   - /flipper_cmd_vel (float64array)

- pub topic:
  - /flipper_states (sensor_msgs/JointState)

## 認識、推定系(Perception & L11n)

KISS-ICP & EKFのコンボで殴る

### sensor_gateway (rust)

STM32 Parser.
stm32からのimuデータを処理し、publishする
microrosは不要だと判断。
プロトコルは自作
TimeStamp注意！！
カルマンフィルタを回すか、線型回帰
通信遅延ゆらぎ(Jitter)にちょっと注意

- pub topic:
   - /imu/data_raw (sensor_msgs/Imu)

### velodyne_driver (C++既存)

Lidarの点群を吐く

 - pub topic:
   - /velodyne_points (sensor_msgs/PointCloud2)
     - 3D点群 

### KISS-ICP (C++既存)

多分最強のLidarOdom
軽い、高精度
3D点群で自己位置推定

- sub topic:
  - /velodyne_points

- pub topic:
    - /odom_lidar (nav_msgs/Odometry)
      - ロボットがどう動いたかのメインの情報源

### robot_localization (c++既存)

センサ融合. EKFノード
IMU & LidarOdomのfusion, encoderは使わない気がする(精度が..)
odom_wheelのCovarianceはかなり大きく設定することになると思う
エンコーダを信じるな

- sub topic:
  - /imu/data_raw
    - 姿勢の安定化
  - /odom_wheel
    - クローラ回転量(使わないかも)
  - /odom_lidar
    - KISS-ICPの推定値

- pub topic:
  - /odometry/filtered

output tf:
  odom -> base_link

### pointcloud_to_laserscan (C++既存)

3D点群を2Dスキャンに潰す
(SLAM_TOOLBOX用)

- sub topic:
  - /velodyne_points

- pub topic:
  - /scan (sensor_msgs/LaserScan)
    - slam_toolboxが2DSLAMなので必要


### SLAM_TOOLBOX (C++既存)

SLAM野郎
2D地図を作成する& 自己位置補正
地図生成は計算コストが低い2DMAPにする

- sub topic:
  - /scan (sensor_msgs/LaserScan)

- output tf:
  map -> odom

## アーム制御系 (Arm Control)


### arm_controller (Rust)

- lib:
  - k (IK/FK)
  - urdf-rs
  - rtt or trajectory_planner
  - gear (衝突判定)


- Arch:
  - TeleopMode
    - Joystickの変位を手先目標速度に変換し、飛んでくるため
    Jacobianを計算し、関節角度に変換
    - 特異点処理(DLSとか)
    - これで計算したcmd_velをドライバに投げる
  - Collision check
    - リンク座標を取得
    - 
  



### gripper_driver (Rust)

Dynamixel制御
電流位置制御したい

- sub topic:
  - /gripper_cmd

- pub topic:
  - /gripper_status
    - 掴んでるか、空振りか、過負荷かなどの状態


## 視覚系(Vision)

### vision_processor (python)

WeChat QrDetectorは文句無しで最強。Zbarなんて使うな
画像圧縮転送もするかも
QR Detectionは絶対にEdge(ロボット側)でやるべき
帯域制御をするべきかもしれない

圧縮はJPEG使うが、
Qualityはdefault(80)より落としてもいいかも
QR検出は30fpsくらいで回す予定
画像転送するときは間引く

- lib:
  - opencv(contrib)
  - cv_bridge

- sub topic:
  - /image??

- pub topic:
  - /qt_codes (std_msgs/String)
    - 検出したQRの内容
  - /image/compressed (sensor_msgs/CompressedImage)


## QoSについて

センサ系は必ずReliability: BEST_EFFORT(UDP的な挙動)に設定すること
RELIABLEがデフォルトだが、遅延が増える
Durability: VOLATILE


## 懸念

通信負荷
時刻同期
