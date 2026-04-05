# Operator Station ワークスペース

Foxglove Studio と `joy_node` を使って遠隔からロボットを操作・モニタリングするための ROS 2 ワークスペースです。

## 構成

```
operator_ws/
├── launch/
│   └── operator.launch.py      # メイン: joy_node + foxglove_bridge + 音声を一括起動
├── src/
│   ├── audio_bridge/           # → main_ws/src/audio_bridge へのシンボリックリンク
│   └── custom_interfaces/      # → main_ws/src/custom_interfaces へのシンボリックリンク
└── zenoh_ope.json5             # Zenoh 接続設定（Robot IP を要設定）
```

## 音声・映像の仕組み

**WebRTC は使用していません。** Zenoh（QUIC）トランスポート上に直接 ROS 2 トピックを流します。

| 種別 | Robot → Operator | Operator → Robot |
|------|-----------------|-----------------|
| 音声 | `/robot/audio` (Opus 32kbps) | `/operator/audio` (Opus 32kbps) |
| 映像 | `/camera/image_raw/ffmpeg` (H.264 BASELINE 2Mbps) | — |

## セットアップ

### 0. Nix キャッシュ有効化（推奨）

operator環境はルート `flake.nix` の `devShells.operator` に統合済みです。
次のように `--accept-flake-config` を付けて入るとバイナリキャッシュが効きます。

```bash
cd operator_ws
nix develop --accept-flake-config ..#operator
```

CI/開発者がキャッシュを温める場合は、リポジトリルートで以下を実行します。

```bash
just operator-cachix
```

### 2. ロボット IP の設定

`zenoh_ope.json5` を開き、ロボット (NUC) の実際の IP アドレスに書き換えます:

```json5
{
  mode: "client",
  connect: { endpoints: ["quic/<ROBOT_IP>:7447"] },  // ← ここを変更
  scouting: { multicast: { enabled: false } },
  transport: { 
    shared_memory: { enabled: true },
    link: { tls: { root_ca_certificate: "quic/server.crt" } }
  }
}
```

### 3. mTLS 証明書の取得

Zenoh は mTLS（相互認証）を使用してセキュアな通信を行います。ロボット側で生成された CA 証明書をオペレータ側にコピーする必要があります。
以下の `Justfile` コマンドで簡単に取得できます。

```bash
cd operator_ws
just get-cert <ROBOT_IP>  # 例: just get-cert 10.42.0.1
```

### 4. 起動

```bash
cd operator_ws
just launch
```

音声なしで起動する場合:
```bash
ros2 launch launch/operator.launch.py use_audio:=false
```

起動後:
1. Foxglove Studio を開き、`ws://127.0.0.1:8765` に接続
2. カメラ映像: `+` → Video パネル → `/camera/image_raw/foxglove_video`
  （または `/camera_front/compressed_video`）を選択 (H.264)
3. PS4 コントローラで操作開始

### 5. トピックの確認

手動で ROS 2 コマンドを実行したい場合は、環境変数を設定します。

```bash
# 現在のターミナルに Zenoh の設定を適用
eval $(just set-env)

# トピックの一覧を確認
just topics
```

## 起動されるノード

| ノード | 役割 |
|--------|------|
| `joy_node` | PS4 コントローラ入力 → `/joy` トピック配信 |
| `foxglove_bridge` | Zenoh 経由で全 ROS 2 トピックを WebSocket (:8765) で Foxglove Studio に配信 |
| `operator_audio_sender` | オペレータ PC マイク → Opus → `/operator/audio` (Zenoh 経由で NUC へ) |
| `operator_audio_receiver` | `/robot/audio` → Opus デコード → オペレータ PC スピーカー |

## Zenoh 通信

```
Operator PC (client)                  Robot NUC (router)
  zenoh_ope.json5                       zenoh_router.json5
  mode: client                          mode: router
  connect → quic/<ROBOT_IP>:7447        listen ← quic/0.0.0.0:7447
```

- `/joy` → Zenoh 経由 → ロボット側 `joy_controller`
- `/camera/image_raw/foxglove_video` / `/camera_front/compressed_video`
  ← Zenoh 経由 ← ロボット側カメラ

## トラブルシューティング

| 症状 | 確認事項 |
|------|---------|
| Zenoh が接続しない | `zenoh_ope.json5` の IP アドレス、ロボット側 zenohd が起動しているか |
| `/joy` がロボットに届かない | `RMW_IMPLEMENTATION=rmw_zenoh_cpp` が設定されているか |
| カメラ映像が出ない | ロボット側 `camera.launch.py` が起動しているか、`/camera/image_raw/foxglove_video` または `/camera_front/compressed_video` が存在するか |
| 音声が聞こえない | `use_audio:=false` になっていないか、PulseAudio デバイスが正しいか |
