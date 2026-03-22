# Operator Station (ROS2 Workspace)

Foxglove Studio と `joy_node` を使って遠隔からロボットを操作・モニタリングするための最小構成と手順をまとめます。

このフォルダは ROS2 ワークスペース形式に整備されています。

構成:
- `src/` — ROS2 パッケージを配置するディレクトリ
- `launch/` — ワークスペースレベルの launch ファイル（パッケージ内の launch を参照するラッパーとして利用できます）

従来の `operator/` と同等のファイルをこのワークスペースに移動しました。

実行例（ワークスペースルートで）:

```bash
# dev shell がある場合
# nix develop

# republish_h264 を実行する（ROS2 環境をソース済み）
python3 republish_h264.py --in-topic /camera/image_raw/ffmpeg --out-topic /camera/image_raw
```

詳細は個別ファイルのヘッダやパッケージ README を参照してください。
