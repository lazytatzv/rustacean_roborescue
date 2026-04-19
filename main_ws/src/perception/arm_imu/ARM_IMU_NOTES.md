# ARM IMU (HWT901B) NOTES
********
- Time Output, Atmospheric Pressure and Height Outputは記録していない
- Sleep/Wake up
********

# 1. 特徴
- 高精度のマイクロプロセッサやカルマンフィルタをもちいることで、安定した姿勢推定が可能。
- データの配列は11個あり最後はSUM値となっている。
- 設定をした後はSAVEコマンドを送って保存し、電源を切るとその設定が反映される。
- IMUを用いる前に校正を行わないといけない

===

## 2. 角度の定義
- X 軸回り: `roll`
	- 機体が前後方向（X 軸）を中心に左右へ傾く回転。
- Y 軸回り: `pitch`
	- 機体が左右方向（Y 軸）を中心に前後へうなずく回転。
- Z 軸回り: `heading`（yaw）
	- 機体が鉛直方向（Z 軸）を中心に水平方向へ向きを変える回転。

===

## 3. 通信設定
- ボーレート(default): 9600 bps (max : 921600 bps)
- 出力レート: 0.2 ~ 200 Hz

===

## 4. 座標系と回転順序
- 座標系: ENU (East-North-Up)
	- X 軸: 右向き
	- Y 軸: 前向き
	- Z 軸: 上向き
- 回転順序: ZYX (yaw-pitch-roll)

===

## 5. PIN
- VCC(RED): 3.3V~5V
- RX(GREEN): connected with TX (HWT901B からのデータ送信)
- TX(YELLOW): connected with RX (HWT901B へのデータ受信)
- GND(BLACK): GND

===

## 6.1. Acceleration Output
```bash
0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM
```
- L, H: 低位バイト、上位バイト
- Tは温度データで、SUMは全てのバイトの合計値。
-  16進数送られてくるし正負があるので、short型(16bitで符号付き整数)に変換してから扱う必要がある

Ax = ((AxH << 8) | AxL) / 32768 * 16 * g
Ay = ((AyH << 8) | AyL) / 32768 * 16 * g
Az = ((AzH << 8) | AzL) / 32768 * 16 * g

===

## 6.2. Angular Velocity Output
```bash
0x55 0x52 WxL WxH WyL WyH WzL WzH TL TH SUM
```
- Wx = ((WxH << 8) | WxL) / 32768 * 2000[deg/s]
- Wy = ((WyH << 8) | WyL) / 32768 * 2000[deg/s]
- Wz = ((WzH << 8) | WzL) / 32768 * 2000[deg/s]

**一秒間に何回転したのかが測れるので、角速度を積分することで角度を求めることも可能。(MAX :  約5.5回転/秒)**

===

## 6.3. Angle Output
```bash
0x55 0x53 RollL RollH PitchL PitchH YawL YawH TL TH SUM
```

- Roll = ((RollH << 8) | RollL) / 32768 * 180 * [deg]
- Pitch = ((PitchH << 8) | PitchL) / 32768 * 180 * [deg]
- Yaw = ((YawH << 8) | YawL) / 32768 * 180 * [deg]

- X軸 : 右方向が軸なので前後の傾きがRoll (±180度)
- Y軸 : 前方向が軸なので左右の傾きがPitch (±90度)
- Z軸 : 上方向が軸なので水平方向の向きがYaw (±180度)

- **回す順番によって最終の向いている向きが変わってしまうため回転順序はオイラー角のZ-Y-Xであることに注意。**

- **Y軸(Pitch)の値が大きくなるとRollとYawの値が不安定になるため、Pitchが90度に近づくような姿勢ではRollとYawの値はあまり信用できない。(簡単に言うと軸が別の軸と重なってしまう)**
**Y軸(Pitch角)が90度を超えると、Pitchの数値は90度より小さい値となりX軸(Roll角)の数値が180度を超えるようになる(例: Pitchが100度のとき、Pitchは80度、Rollは190度のような値になる、多分、、)**

- **Roll, Pitch, Yawは加速度センサとジャイロセンサのデータを融合してカルマンフィルタで推定されているため、角速度を積分して求める方法よりも安定した値が得られる。**

===

## 6.4. Magnetic Output
```bash
0x55 0x54 MxL MxH MyL MyH MzL MzH TL TH SUM
```
- Mx = ((MxH << 8) | MxL)[gauss]
- My = ((MyH << 8) | MyL)[gauss]
- Mz = ((MzH << 8) | MzL)[gauss]

## 6.5. Quaternion
```bash
0x55 0x59 Q0L Q0H Q1L Q1H Q2L Q2H Q3L Q3H SUM
```

- Q0 = ((Q0H << 8) | Q0L) / 32768
- Q1 = ((Q1H << 8) | Q1L) / 32768
- Q2 = ((Q2H << 8) | Q2L) / 32768
- Q3 = ((Q3H << 8) | Q3L) / 32768

- **Quaternionとは具体的に言うと、4次元のベクトルで回転を表現する方法で、オイラー角のような特定の軸に依存しない回転表現。**

- **Quaternionはオイラー角のような特定の軸に依存しない回転表現であるため、複数の回転を組み合わせる際に計算が簡単になる。**

- **Quaternionはオイラー角のような特定の軸に依存しない回転表現であるため、回転の補間やスムージングが容易になる。**

===

## 7. Config command
```bash
0xFF 0xAA Address DataL DataH
```
- SAVE : 0x00 current config, 0x01 default confi
- Calibrate : 0x00 acc & gyro, 0x01 mag, 0x02 exit, 0x03 height reset
```bash
1  加速度・ジャイロ校正
2 磁気校正
0 校正モード終了
3 高度リセット
```

- Install Direction : 0x00 horizon, 0x01 vertical
- Algorithm Transition : 0x00 9-axis, 0x01 6-axis
- Gyroscope Automatic Calibration : 0x00 able, 0x01 disable(数秒まっていたらそこを基準にして自動でジャイロのオフセットを補正してくれる機能)
- Return Content : 何のデータを返すかを設定する（資料を見よ）
- Return Rate : データの出力レートを設定する（資料を見よ）
- Baud Rate : Baud Rateを設定
