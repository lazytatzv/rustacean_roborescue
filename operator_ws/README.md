# Operator Station ワークスペース

Foxglove Studio と `joy_node` を使って遠隔からロボットを操作・モニタリングするための ROS 2 ワークスペースです。

## 構成

```
operator_ws/
├── launch/
│   ├── operator.launch.py          # メイン: joy_node + foxglove_bridge を一括起動
│   └── operator_webrtc.launch.py   # WebRTC 経由映像受信を使う場合（省略可）
├── src/
│   ├── gst_webrtc_operator/        # WebRTC オペレータノード（オプション）
│   └── h264_republisher/           # H.264 → raw Image 変換（デバッグ用）
└── zenoh_ope.json5                 # Zenoh 接続設定（Robot IP を要設定）
```

## 前提

- ROS 2 Jazzy がインストールされていること
- `rmw_zenoh_cpp`, `foxglove_bridge`, `joy` パッケージが利用可能なこと
- PS4 (DualShock 4) コントローラが `/dev/input/js0` に認識されていること

## セットアップ

### 1. Robot IP の設定

`zenoh_ope.json5` を開き、ロボット (NUC) の実際の IP アドレスに書き換えます:

```json5
{
  mode: "client",
  connect: { endpoints: ["quic/<ROBOT_IP>:7447"] },  // ← ここを変更
  scouting: { multicast: { enabled: false } },
  transport: { shared_memory: { enabled: true } }
}
```

### 2. 起動

```bash
cd operator_ws
ros2 launch launch/operator.launch.py
```

起動後:
1. Foxglove Studio を開き、`ws://127.0.0.1:8765` に接続
2. カメラ映像: `/camera/image_raw/ffmpeg` トピックを追加 (H.264)
3. PS4 コントローラで操作開始（[コントローラ操作方法](../docs/OPERATION.md#7-コントローラ操作)参照）

## 起動されるノード

| ノード | 役割 |
|--------|------|
| `joy_node` | PS4 コントローラ入力 → `/joy` トピック配信 |
| `foxglove_bridge` | Zenoh 経由で受信した全 ROS 2 トピックを WebSocket (:8765) で Foxglove Studio に配信 |

## Zenoh 通信

```
Operator PC (client)                  Robot NUC (router)
  zenoh_ope.json5                       zenoh_router.json5
  mode: client                          mode: router
  connect → quic/<ROBOT_IP>:7447        listen ← quic/0.0.0.0:7447
```

- `/joy` トピックはオペレータ側で publish → Zenoh 経由でロボット側の `joy_controller` に届く
- カメラ映像 (`/camera/image_raw/ffmpeg` など) はロボット側で publish → Zenoh 経由でオペレータ側の `foxglove_bridge` に届く

## H.264 映像の表示

Foxglove Studio でカメラ映像を見るには:

1. 「+」ボタン → 「Image」パネルを追加
2. トピックに `/camera/image_raw/ffmpeg` を選択

Foxglove Studio は H.264 (BASELINE プロファイル、B フレームなし) をネイティブデコードします。

> **Note**: ロボット側の `camera.launch.py` が `ffmpeg_image_transport` で H.264 エンコードしているため、
> 生の `/camera/image_raw` より帯域を大幅に節約できます。

## デバッグ用: H.264 → raw 変換

H.264 を扱えないツールが必要な場合:

```bash
# operator_ws/src/h264_republisher/ をビルドしてから
ros2 run h264_republisher republish_h264 \
  --in-topic /camera/image_raw/ffmpeg \
  --out-topic /camera/image_raw_decoded
```

## トラブルシューティング

| 症状 | 確認事項 |
|------|---------|
| Zenoh が接続しない | `zenoh_ope.json5` の IP アドレス、ロボット側 zenohd が起動しているか |
| `/joy` がロボットに届かない | `RMW_IMPLEMENTATION=rmw_zenoh_cpp` が設定されているか |
| カメラ映像が出ない | ロボット側 `camera.launch.py` が起動しているか、`/camera/image_raw/ffmpeg` トピックが存在するか |
| Foxglove に接続できない | `foxglove_bridge` が `127.0.0.1:8765` で起動しているか |
