# Operator Station
#
# Foxglove Studio + joy_node だけでロボットを遠隔操縦するための最小環境。
#
# ## セットアップ
#
# ### 1. Foxglove Studio (GUI モニタ)
#
# https://foxglove.dev/download からダウンロード (Electron アプリ, Nix 不要)
# または:
# ```
# sudo snap install foxglove-studio     # Ubuntu
# flatpak install flathub dev.foxglove.studio  # Flatpak
# ```
#
# ### 2. joy_node (コントローラ入力)
#
# ```bash
# cd operator/
#
# # Zenoh 設定: <ROBOT_IP> を NUC の IP に書き換え
# vim zenoh_ope.json5
#
# # Nix 環境に入る
# nix develop --impure
#
# # PS4 コントローラを USB/Bluetooth 接続後
# ros2 run joy joy_node
# ```
#
# ### 3. Foxglove Studio で接続
#
# 接続先: `ws://<ROBOT_IP>:8765`
#
# ## 確認
#
# | 確認項目 | コマンド |
# |----------|---------|
# | joy_node がトピック出力 | `ros2 topic echo /joy --once` |
# | ロボット側ノードが見える | `ros2 node list` |
# | Zenoh 接続状態 | `ros2 topic list` でロボット側トピックが見えること |
