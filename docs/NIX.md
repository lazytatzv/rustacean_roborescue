### Nix 環境セットアップ

#### 前提条件

- Nix 2.18 以上 (flakes 有効)
- GPU 利用時は nixGL が必要 (`rviz2`, `gz sim` 等の OpenGL アプリ)

#### 初回セットアップ

```bash
git clone --recursive <repourl>
cd rustacean_roborescue

# git submodule update --init --recursive
echo 'direnv hook fish | source' >> ~/.config/fish/config.fish
source ~/.config/fish/config.fish

nix profile install nixpkgs#direnv nixpkgs#nix-direnv
mkdir -p ~/.config/direnv
echo 'source_url "https://raw.githubusercontent.com/nix-community/nix-direnv/master/direnvrc" "sha256-t/qsHuSymixvs+MpxeBGDe0ViFBUrbij5Z9Xy1Bgb28="' > ~/.config/direnv/direnvrc
```

direnv allow が必要

#### Nix 開発シェル (direnv なし)

```bash
just nix
# または: nix develop --impure
```

> `--impure` が必要な理由: nixGL が `/run/opengl-driver` 等のホスト側 GPU ドライバを参照するため。

#### flake.nix で追加される主要ツール

| カテゴリ | パッケージ例 |
|----------|-------------|
| ROS 2 Jazzy | rclcpp, rclpy, rclrs, nav2, slam-toolbox, ros-gz-sim, foxglove-bridge |
| Rust | nightly ツールチェーン, cargo-ament-build |
| シミュレーション | Gazebo Harmonic (gz sim 8) |
| 可視化 | rviz2, joint-state-publisher-gui, plotjuggler |
| ビルド | colcon, cmake, ninja, clang, mold |

#### nixGL のメモ

Nix 環境では OpenGL アプリ (rviz2, gz sim) は直接使うと GLX エラーが出る場合がある。
`nixGL` 経由で起動する:

```bash
nixGL rviz2
nixGL gz sim
```

`flake.nix` で `alias gz="nixGL gz"` が設定されているため Gazebo は通常通り使える。
`display.launch.py` は `use_rviz:=false` 引数で RViz なし起動が可能。


