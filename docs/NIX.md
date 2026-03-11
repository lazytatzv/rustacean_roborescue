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

#### Nav2 パッケージ一覧 (flake.nix)

Jazzy では navigation2 メタパッケージが使えないため、以下の個別パッケージを flake.nix で指定:

- ros.navigation2
- ros.nav2-common
- ros.nav2-core
- ros.nav2-util
- ros.nav2-msgs
- ros.nav2-map-server
- ros.nav2-controller
- ros.nav2-planner
- ros.nav2-behaviors
- ros.nav2-bt-navigator
- ros.nav2-velocity-smoother
- ros.nav2-lifecycle-manager
- ros.nav2-costmap-2d
- ros.nav2-regulated-pure-pursuit-controller
- ros.nav2-smac-planner
- ros.nav2-rviz-plugins

#### GPU/nixGL 注意

- Gazebo/RViz2 のセンサレンダリング (ogre2/ogre) は **GPU + ディスプレイ必須**
- ヘッドレス環境では sensors-system プラグインを SDF でコメントアウトする (デフォルトで対応済み)
- 物理シミュレーション・差動駆動・TF は GPU なしで動作
- `nixGL` 経由で GUI を起動する場合は `ros2 launch bringup simulation.launch.py headless:=false` を推奨


