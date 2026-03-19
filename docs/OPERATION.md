# 操作マニュアル

## 目次

1. [環境構築](#1-環境構築)
2. [ビルド](#2-ビルド)
3. [実機起動 (ロボット側)](#3-実機起動-ロボット側)
4. [オペレータ PC の起動](#4-オペレータ-pc-の起動)
5. [ネットワーク構成 (Zenoh)](#5-ネットワーク構成-zenoh)
6. [シミュレーション (Gazebo)](#6-シミュレーション-gazebo)
7. [コントローラ操作](#7-コントローラ操作)
8. [自律走行 (Nav2)](#8-自律走行-nav2)
9. [シリアルデバイス設定](#9-シリアルデバイス設定)
10. [トラブルシューティング](#10-トラブルシューティング)

---

## 1. 環境構築

### 1.1 前提条件

- **OS**: Linux (NixOS / Ubuntu 等)
- **Nix**: バージョン 2.18 以上 (flakes 有効)
- **GPU**: Gazebo GUI を使う場合は OpenGL 3.3 以上 (nixGL 経由)

### 1.2 初回セットアップ

```bash
# リポジトリクローン (サブモジュール含む)
git clone --recursive <REPO_URL>
cd rustacean_roborescue

# サブモジュール取得
just sync
```

---

## 2. ビルド

### 2.1 フルビルド

```bash
cd main_ws
just forge
```

---

## 3. 実機起動 (ロボット側)

### 3.1 Zenoh 設定の確認
`main_ws/src/bringup/config/zenoh_robot.json5` が `mode: "peer"` であることを確認。

### 3.2 起動
```bash
cd main_ws
source install/setup.bash
ros2 launch bringup system.launch.py
```

---

## 4. オペレータ PC の起動

### 4.1 ネットワーク設定
`operator/zenoh_ope.json5` の `connect.endpoints` を、ロボットの実際の IP アドレスに書き換える。

### 4.2 起動
```bash
cd operator
nix develop
# 別ターミナル
ros2 run joy joy_node
# 別ターミナル
ros2 run foxglove_bridge foxglove_bridge
```

---

## 6. シミュレーション (Gazebo)

実機がなくても、Gazebo Harmonic を使用して自律走行やアームの動作をテスト可能です。

### 6.1 基本的な起動
```bash
# シミュレーション + SLAM (地図作成)
ros2 launch bringup simulation.launch.py use_slam:=true
```

### 6.2 自律走行 (Nav2) を有効にして起動
```bash
# シミュレーション + SLAM + Nav2
ros2 launch bringup simulation.launch.py use_slam:=true use_nav2:=true
```

---

## 7. コントローラ操作

PS4 コントローラで操作します。

| ボタン | モード | 説明 |
|--------|--------|------|
| **PS** | STOP | **緊急停止** (全出力遮断) |
| **OPTIONS** | DRIVE | 走行・フリッパ操作 |
| **SHARE** | **ARM / JOINT** | アーム操作 (IK / 関節個別) |

---

## 8. 自律走行 (Nav2)

1. **環境の起動**: 6.2 の手順でシミュレーションを起動。
2. **地図生成**: ロボットをコントローラで少し走らせ、RViz 上で周囲の地図が描画されるのを待つ。
3. **目的地の指定**: RViz 上部の「Nav2 Goal」ボタンを押し、地図上の行きたい場所をクリック＆ドラッグ。
4. **自動走行**: Nav2 が経路を計算し、ロボットが自動で障害物を避けながら目的地まで走行します。

---

## 10. トラブルシューティング

### 10.1 通信がつながらない
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` が設定されているか確認。
- `zenoh_ope.json5` の IP アドレスを確認。

### 10.2 Nav2 が経路を作らない
- 地図が十分にできていないか、目的地が障害物に近すぎる可能性があります。
- RViz 上でコストマップ（赤い影）を確認してください。
