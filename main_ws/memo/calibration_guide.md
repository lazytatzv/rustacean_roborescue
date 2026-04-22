# 🔧 キャリブレーション・ガイド

このガイドでは、ロボットの各部位を正しく動かすための調整手順をまとめます。

## 1. アーム & グリッパー (Dynamixel)

### ゼロ点オフセット (Arm Offsets)
URDF上の 0 rad（例：腕をまっすぐ伸ばした状態）と、実際のサーボの 0 rad がずれている場合に調整します。
1.  `PS`ボタンで脱力させるか、電源を切って手動でアームを **URDFのゼロ姿勢**（各関節が真っ直ぐな状態）にする。
2.  `ros2 topic echo /joint_states` を実行。
3.  表示される各関節の `position` の値を読み取り、その **符号を反転させた値** を `arm_gripper_driver.yaml` の `arm_offsets` に記入する。
    *   例: 表示が `0.15` なら、Offsetは `-0.15`。

### グリッパー開閉範囲 (Gripper Positions)
1.  `ros2 topic echo /gripper_status` を実行。
2.  手動でグリッパーを **理想の全開位置** まで広げる。
3.  表示される `position` (Ticks) を `joy_controller.yaml` の `gripper_open_position` に記入。
4.  手動でグリッパーを **理想の全閉位置** まで閉じる。
5.  表示される `position` (Ticks) を `joy_controller.yaml` の `gripper_close_position` に記入。

---

## 2. 走行 (Roboclaw)

### 距離・速度の正確性 (counts_per_meter)
1.  ロボットを床に置き、1メートル進ませる。
2.  実際に進んだ距離を測る。
3.  もし 1.1m 進んでしまったら、今の `counts_per_meter` に `(1.1 / 1.0)` を掛けて補正する。

---

## 3. フリッパー (Dynamixel Wheel Mode)

### 回転方向
1.  フリッパーを回してみて、逆方向に回る場合は `flipper_driver.yaml` の `servo_inverted` を `true` または `false` に切り替える。
2.  Foxglove上の表示と回転が逆の場合は、URDFの軸定義を見直す。

---

## 4. 把握検出 (Grasp Detection)
1.  `ros2 topic echo /gripper_status` を見ながら、物を掴んでみる。
2.  物を掴んで止まった瞬間の `current` (mA) を読み取る。
3.  その値より少し低い値を `joy_controller.yaml` の `gripper_effort_threshold` に設定する。
    *   空振りで止まる場合は値を上げ、物が潰れる場合は値を下げる。
