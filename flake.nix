{
  description = "Rustacean RoboRescue — レスキューロボット開発環境";

  # ── Binary cache ─────────────────────────────────────────────────────────
  # CI・ローカルともにこのキャッシュを参照することで nix-ros-overlay の
  # 再ビルドを避け、初回でも数分でシェルに入れるようにする。
  # GitHub Actions では `--accept-flake-config` を渡すことで有効になる。
  nixConfig = {
    extra-substituters = [
      "https://roborescue-nix.cachix.org"
      "https://nix-community.cachix.org"
      "https://ros.cachix.org"
    ];
    extra-trusted-public-keys = [
      "roborescue-nix.cachix.org-1:qy3rP4VwHob/xePMW77gUxZVvPMz8izs86rIdruro0U="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
      "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
    ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    nixgl.url = "github:nix-community/nixGL";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, rust-overlay, flake-utils, nixgl }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            (import rust-overlay)
            # opencv4 に WITH_PROTOBUF=ON を強制する
            # protobuf_29 は既にbuild inputsにあるが、protobuf 25+でcmake package名が
            # 変わりauto-detectionが失敗するため明示指定が必要。
            # → DNN の Caffe モデル読み込みが有効になり WeChatQRCode が動作する
            (final: prev: {
              opencv4 = prev.opencv4.overrideAttrs (old: {
                cmakeFlags = old.cmakeFlags ++ [ "-DWITH_PROTOBUF=ON" ];
              });
            })
          ];
          config.allowUnfree = true;
          config.permittedInsecurePackages = [
            "freeimage-3.18.0-unstable-2024-04-18"
          ];
        };

        ROS_VERSION = "jazzy";
        ros = pkgs.rosPackages.${ROS_VERSION};
        tbbRuntime = if pkgs ? onetbb then pkgs.onetbb else pkgs.tbb;

        # ── Rust ツールチェーン ──────────────────────────────
        # nightly: proc-macro / bindgen に必要
        # targets: stm32_ws の STM32F4 ベアメタル用
        rustNightly = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "llvm-tools-preview" ];
          targets = [ "thumbv7em-none-eabihf" ];
        };

        # ── ビルドツール & CLI ──────────────────────────────
        buildTools = with pkgs; [
          # --- C++ ビルド ---
          cmake ccache sccache mold ninja
          clang-tools clang llvmPackages.openmp
          boost                       # crawler_driver (boost::asio)
          tbbRuntime                  # kiss_icp runtime (libtbb.so.12)

          # --- Rust ---
          rustNightly
          bacon                       # バックグラウンド コンパイルチェック
          cargo-watch cargo-expand cargo-binutils
          cargo-ament-build           # ROS 2 Rust パッケージビルド
          probe-rs                    # STM32 書き込み・デバッグ

          # --- Zenoh (RMW ミドルウェア) ---
          zenoh
          ros.rmw-zenoh-cpp

          # --- Python ---
          python3
          uv
          python3Packages.numpy
          python3Packages.opencv4     # qr_detector / vision_processor
          python3Packages.black
          python3Packages.isort
          python3Packages.ipdb
          pkg-config
          ffmpeg

          # --- GStreamer (audio_bridge: Opus 双方向音声) ---
          gst_all_1.gstreamer
          gst_all_1.gst-plugins-base  # audioconvert, audioresample, appsrc/sink, opusenc/dec
          gst_all_1.gst-plugins-good  # pulsesrc, pulsesink
          python3Packages.pygobject3  # gi.repository.Gst (GStreamer Python バインディング)

          # --- colcon ---
          colcon
          python3Packages.colcon-cargo
          python3Packages.colcon-ros-cargo

          # --- Shell & DevTools ---
          fish fishPlugins.bass just
          git lazygit ripgrep fd btop zellij tmux
          nodePackages.mermaid-cli    # topology/*.mmd 図の生成

          # --- ROS 2 ビルドインフラ (buildTools 側に置く) ---
          ros.ros-core
          ros.ament-cmake

          vtk
          pcl
        ];

        # ── ROS 2 パッケージ群 ──────────────────────────────
        # symlinkJoin で1ディレクトリに圧縮し PATH 爆発を防ぐ
        rosDeps = [
          # --- メッセージ生成 (custom_interfaces) ---
          ros.rosidl-default-generators

          # --- メッセージ型 ---
          ros.std-msgs
          ros.sensor-msgs
          ros.geometry-msgs
          ros.nav-msgs
          ros.visualization-msgs
          ros.std-srvs
          ros.example-interfaces      # rclrs が link 時に必要

          # --- TF2 (spark_fast_lio / perception.launch.py) ---
          ros.tf2                     # core tf2 library
          ros.tf2-msgs                # tf2 message types
          ros.tf2-ros                 # static_transform_publisher
          ros.tf2-ros-py              # Python bindings for tf2_ros (TransformBroadcaster etc.)
          ros.tf2-py                  # tf2_py C extension (required by tf2_ros_py)
          ros.tf2-eigen
          ros.tf2-sensor-msgs
          ros.tf2-geometry-msgs
          ros.orocos-kdl-vendor       # transitive dep of tf2_geometry_msgs
          ros.eigen3-cmake-module     # transitive dep of orocos_kdl_vendor
          ros.message-filters         # spark_fast_lio dep

          # --- Perception ---
          ros.velodyne                # VLP-16 LiDAR ドライバ (メタパッケージ)
          ros.velodyne-driver
          ros.velodyne-pointcloud
          ros.pointcloud-to-laserscan # 3D → 2D LaserScan
          ros.slam-toolbox            # 2D SLAM
          ros.pcl-ros
          ros.pcl-conversions
          ros.pcl-msgs                # transitive dep of pcl_conversions
          ros.cv-bridge               # OpenCV ↔ ROS (qr_detector)
          ros.image-transport
          ros.compressed-image-transport
          ros.ffmpeg-image-transport       # H.264 compression support
          ros.ffmpeg-image-transport-msgs  # FFMPEGPacket メッセージ型
          ros.foxglove-msgs                # foxglove.CompressedVideo スキーマ (Foxglove Studio ネイティブH264)
          ros.v4l2-camera                  # Optimized camera driver
          ros.camera-info-manager
          ros.rclcpp-components


          #ros.pcl

          # --- Hardware ---
          ros.joy                     # PS4 コントローラ
          ros.joy-linux
          ros.teleop-twist-keyboard   # キーボードでロボット操作
          ros.usb-cam                 # USB カメラ
          ros.dynamixel-workbench-toolbox  # フリッパ / gripper_driver
          ros.dynamixel-sdk
          ros.dynamixel-interfaces
          ros.hardware-interface      # dynamixel_hardware_interface
          ros.pluginlib               # dynamixel_hardware_interface / nav2
          ros.realtime-tools          # dynamixel_hardware_interface
          ros.control-msgs
          ros.generate-parameter-library
          ros.urdfdom
          ros.urdfdom-headers
          ros.control-toolbox
          ros.angles
          ros.filters
          ros.tinyxml2-vendor
          ros.console-bridge-vendor
          ros.diagnostic-updater
          ros.backward-ros
          ros.kdl-parser
          ros.pal-statistics
          ros.pal-statistics-msgs
          ros.controller-interface
          ros.controller-manager
          ros.transmission-interface
          ros.joint-limits

          # --- Simulation (Gazebo Harmonic via ros-gz) ---
          ros.xacro                   # URDF xacro 処理
          ros.robot-state-publisher   # TF from URDF
          ros.urdf                    # URDF パーサ
          ros.urdf-parser-plugin      # robot_state_publisher が pluginlib で要求
          ros.joint-state-publisher   # joint_state_publisher
          ros.joint-state-publisher-gui # GUI スライダでジョイント操作
          ros.ros-gz-sim              # Gazebo Harmonic シミュレータ
          ros.ros-gz-bridge           # ROS 2 ↔ Gazebo トピックブリッジ
          ros.ros-gz-image            # Gazebo カメラ画像 → ROS 2
          ros.ros-gz-interfaces       # Gazebo メッセージ型
          ros.gz-tools-vendor         # gz コマンド (gz sim 等)
          ros.gz-sim-vendor           # Gazebo Harmonic 本体

          # --- Navigation (Nav2) ---
          ros.navigation2             # Nav2 メタパッケージ
          ros.nav2-common             # 共通ユーティリティ
          ros.nav2-core               # プラグインインタフェース (pluginlib)
          ros.nav2-util               # Nav2 ユーティリティ
          ros.nav2-msgs               # Nav2 メッセージ型
          ros.nav2-map-server         # map_server (static map)
          ros.nav2-controller         # controller_server
          ros.nav2-planner            # planner_server
          ros.nav2-behaviors          # behavior_server (spin, backup, wait)
          ros.nav2-bt-navigator       # BehaviorTree ナビゲータ
          ros.nav2-velocity-smoother  # velocity_smoother
          ros.nav2-lifecycle-manager  # lifecycle_manager
          ros.nav2-costmap-2d         # costmap (obstacle/inflation/static)
          ros.nav2-regulated-pure-pursuit-controller  # 経路追従コントローラ
          ros.nav2-smac-planner       # SmacPlanner2D 経路計画
          ros.nav2-rviz-plugins       # rviz2 用 Nav2 Goal ツール

          # --- ノード合成 (spark_fast_lio) ---
          ros.rclcpp-components

          # --- 可視化 & デバッグ ---
          ros.rviz2
          ros.rviz-common
          ros.rviz-rendering
          ros.rviz-default-plugins
          ros.rviz-ogre-vendor
          ros.rviz-assimp-vendor
          ros.point-cloud-transport    # rviz_default_plugins 依存
          ros.laser-geometry           # LaserScan 表示
          ros.interactive-markers      # rviz interactive markers
          ros.resource-retriever       # URDF メッシュ読み込み
          ros.map-msgs                 # Map 表示
          ros.qt-gui
          ros.qt-gui-py-common
          ros.qt-dotgraph
          ros.python-qt-binding
          ros.rqt
          ros.rqt-gui
          ros.rqt-gui-py
          ros.rqt-gui-cpp
          ros.rqt-py-common
          ros.qt-gui-cpp
          ros.rqt-common-plugins
          ros.rqt-topic
          ros.rqt-publisher
          ros.rqt-service-caller
          ros.rqt-tf-tree
          ros.rqt-bag
          ros.rqt-msg
          ros.rqt-srv
          ros.rqt-action
          ros.rqt-shell
          ros.rqt-py-console
          ros.rqt-image-view
          ros.image-view
          ros.ros2topic
          ros.ros2service
          ros.ros2node
          ros.rqt-graph
          ros.rqt-plot
          ros.rqt-console

          # --- Foxglove Studio ブリッジ (WebSocket :8765) ---
          ros.foxglove-bridge
        ];

        # C++ ライブラリ (ROS 外)
        cppLibs = with pkgs; [ eigen orocos-kdl tbbRuntime ];

        # ── CI 専用 ROS 依存 ────────────────────────────────────────────────
        # Gazebo / RViz2 / Nav2 / SLAM など GUI・シミュレーション系を除外し
        # ビルド・テストに必要な最小セットだけを含める。
        # これにより Nix closure が大幅に小さくなりキャッシュヒット率が上がる。
        ciRosDeps = [
          ros.rosidl-default-generators
          ros.std-msgs ros.sensor-msgs ros.geometry-msgs ros.nav-msgs
          ros.visualization-msgs ros.std-srvs ros.example-interfaces
          ros.rcl
          ros.rclcpp ros.rclcpp-components
          ros.tf2 ros.tf2-msgs ros.tf2-ros ros.tf2-ros-py ros.tf2-py
          ros.tf2-eigen ros.tf2-sensor-msgs ros.tf2-geometry-msgs
          ros.orocos-kdl-vendor ros.eigen3-cmake-module ros.message-filters
          ros.cv-bridge ros.image-transport ros.compressed-image-transport
          ros.camera-info-manager ros.image-transport
          ros.ffmpeg-image-transport ros.v4l2-camera
          ros.joy ros.joy-linux
          ros.usb-cam
          ros.dynamixel-workbench-toolbox ros.dynamixel-sdk ros.dynamixel-interfaces
          ros.hardware-interface ros.pluginlib ros.realtime-tools ros.control-msgs
          ros.generate-parameter-library
          ros.urdfdom ros.urdfdom-headers ros.control-toolbox ros.angles ros.filters
          ros.tinyxml2-vendor ros.console-bridge-vendor ros.diagnostic-updater
          ros.backward-ros ros.kdl-parser
          ros.pal-statistics ros.pal-statistics-msgs
          ros.controller-interface ros.controller-manager ros.transmission-interface
          ros.joint-limits
          ros.xacro ros.robot-state-publisher ros.urdf ros.urdf-parser-plugin
          ros.joint-state-publisher
          ros.velodyne ros.velodyne-driver ros.velodyne-pointcloud
          ros.pointcloud-to-laserscan
          ros.pcl-ros ros.pcl-conversions ros.pcl-msgs
          ros.laser-geometry ros.interactive-markers ros.resource-retriever
          ros.map-msgs ros.point-cloud-transport
          ros.libstatistics-collector
          ros.ament-cmake ros.ros-core
          ros.rmw-zenoh-cpp
          # Nav2: インターフェース層のみ (巨大な実行バイナリは除外)
          ros.nav2-msgs ros.nav2-core ros.nav2-util ros.nav2-common
        ];

        ciEnv = pkgs.symlinkJoin {
          name = "roborescue-ci-env";
          paths = ciRosDeps ++ cppLibs;
        };

        # ROS + C++ ライブラリを1ディレクトリに圧縮
        roboRescueEnv = pkgs.symlinkJoin {
          name = "roborescue-compressed-env";
          paths = rosDeps ++ cppLibs;
        };

        # Explicit Python environment with common developer packages (NumPy etc.)
        pythonEnv = pkgs.python3.withPackages (ps: with ps; [ numpy opencv4 pytest ]);

        operatorDeps = [
          ros.ros-core
          ros.rmw-zenoh-cpp
          ros.joy
          ros.joy-linux
          ros.foxglove-bridge
          ros.ros2cli-common-extensions
          ros.rclpy
          ros.rosidl-default-generators
          ros.rosidl-default-runtime
          ros.image-transport
          ros.ffmpeg-image-transport
          ros.ffmpeg-image-transport-msgs
          pkgs.gst_all_1.gstreamer
          pkgs.gst_all_1.gst-plugins-base
          pkgs.gst_all_1.gst-plugins-good
          pkgs.gobject-introspection
          pkgs.python3Packages.pygobject3
          pkgs.pulseaudio
          pkgs.zenoh
        ];

        operatorEnv = pkgs.symlinkJoin {
          name = "roborescue-operator-env";
          paths = operatorDeps;
        };

      in
      {
        devShells.default = pkgs.mkShell {
          name = "RoboRescue Env";

          # rosDeps を直接渡すと PATH が爆発するので圧縮環境のみ渡す
          packages = buildTools ++ [ roboRescueEnv pythonEnv nixgl.packages.${system}.nixGLIntel ];

          shellHook = ''
            # --- ROS 2 ---
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2

            # --- Rust ---
            export RUST_SRC_PATH="${rustNightly}/lib/rustlib/src/rust/library"

            # --- C++ ビルド高速化 ---
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export TBBROOT="${tbbRuntime}"
            export CCACHE_DIR="''${XDG_CACHE_HOME:-$HOME/.cache}/ccache"
            export CCACHE_MAXSIZE="20G"
            export SCCACHE_DIR="''${XDG_CACHE_HOME:-$HOME/.cache}/sccache"
            export CC="ccache clang"
            export CXX="ccache clang++"
            export RUSTFLAGS="-C link-arg=-fuse-ld=mold"

            # Ensure the Nix-managed pythonEnv is first on PATH so `python`
            # resolves to the flake-provided interpreter and its packages
            # (NumPy, OpenCV) are available to CMake/colcon.
            export PATH="${pythonEnv}/bin:$PATH"
            export PYTHON_EXECUTABLE="${pythonEnv}/bin/python"
            # Make site-packages visible to other tools in the shell
            export PYTHONPATH="${pythonEnv}/lib/python${toString pkgs.python3.version}/site-packages:$PYTHONPATH"

            # --- OpenMP (spark_fast_lio 用) ---
            export CFLAGS="-fopenmp $CFLAGS"
            export CXXFLAGS="-fopenmp $CXXFLAGS"
            export LDFLAGS="-fopenmp $LDFLAGS"

            # --- GPU ドライバ パススルー (非-NixOS) ---
            # Intel/AMD (Mesa): nixGLIntel を nixGL として使う (pure, cachix safe)
            # NVIDIA: impure が必要なため flake には含めない。
            #   → nix run --impure github:nix-community/nixGL -- <cmd> を使うか
            #     NIXGL_CMD 環境変数で上書きする。
            _detected_nixgl=""
            if lspci 2>/dev/null | grep -qi nvidia; then
              echo "⚠️  NVIDIA GPU detected. nixGL is not bundled (requires --impure)."
              echo "   Set: export NIXGL_CMD='nix run --impure github:nix-community/nixGL --'"
              _detected_nixgl="''${NIXGL_CMD:-nix run --impure github:nix-community/nixGL --}"
            else
              _detected_nixgl="nixGLIntel"
            fi
            if [ -n "$_detected_nixgl" ]; then
              alias nixGL="$_detected_nixgl"
              alias gz="$_detected_nixgl gz"
              alias rviz2="$_detected_nixgl rviz2"
              alias rqt="$_detected_nixgl rqt"
              alias rqt_graph="$_detected_nixgl rqt_graph"
            fi
            unset _detected_nixgl

            # --- ライブラリパス ---
            # rmw_zenoh_cpp は symlinkJoin 経由だと .so 探索に失敗するケースがあるため
            # 該当パッケージの実体パスを明示的に先頭へ追加する。
            export LD_LIBRARY_PATH="${tbbRuntime}/lib:${ros.rmw-zenoh-cpp}/lib:${ros.rmw-zenoh-cpp}/lib64:${ros.zenoh-cpp-vendor}/lib:${ros.zenoh-cpp-vendor}/lib64:${roboRescueEnv}/lib:${roboRescueEnv}/lib64''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            #export CMAKE_PREFIX_PATH="${roboRescueEnv}:${pkgs.vtk}/lib/cmake/vtk:$CMAKE_PREFIX_PATH"
            export CMAKE_PREFIX_PATH="${roboRescueEnv}:$CMAKE_PREFIX_PATH"
            export AMENT_PREFIX_PATH="${roboRescueEnv}:$AMENT_PREFIX_PATH"

            # --- ROS 2 RMW (Zenoh) ---
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            # どの cwd から nix develop しても見つかるように解決する
            _repo_root=""
            if command -v git >/dev/null 2>&1; then
              _repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
            fi
            if [ -z "$_repo_root" ] && [ -f "$PWD/flake.nix" ]; then
              _repo_root="$PWD"
            fi
            if [ -n "$_repo_root" ] && [ -f "$_repo_root/main_ws/src/bringup/config/zenoh_robot.json5" ]; then
              export RMW_ZENOH_CONFIG_URI="file://$_repo_root/main_ws/src/bringup/config/zenoh_robot.json5"
            fi
            unset _repo_root

            # --- Gazebo Harmonic (gz sim) リソースパス ---
            export GZ_SIM_RESOURCE_PATH="${roboRescueEnv}/share:$GZ_SIM_RESOURCE_PATH"

            echo "======================================================="
            echo " Rustacean RoboRescue Dev Environment"
            echo " ROS: ${ROS_VERSION} | Rust: nightly | Sim: Gazebo Harmonic"
            echo "======================================================="
          '';
        };

        devShells.operator = pkgs.mkShell {
          name = "RoboRescue Operator";
          packages = [ operatorEnv pkgs.uv ];

          shellHook = ''
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
            export LD_LIBRARY_PATH="${ros.rmw-zenoh-cpp}/lib:${ros.rmw-zenoh-cpp}/lib64:${ros.zenoh-cpp-vendor}/lib:${ros.zenoh-cpp-vendor}/lib64''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

            _repo_root=""
            if command -v git >/dev/null 2>&1; then
              _repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
            fi
            if [ -z "$_repo_root" ] && [ -f "$PWD/flake.nix" ]; then
              _repo_root="$PWD"
            fi

            _op_cfg=""
            if [ -n "$_repo_root" ] && [ -f "$_repo_root/operator_ws/zenoh_ope.json5" ]; then
              _op_cfg="$_repo_root/operator_ws/zenoh_ope.json5"
            elif [ -f "$PWD/zenoh_ope.json5" ]; then
              _op_cfg="$PWD/zenoh_ope.json5"
            fi

            if [ -n "$_op_cfg" ]; then
              export RMW_ZENOH_CONFIG_URI="file://$_op_cfg"
              echo "✅ Zenoh config: $_op_cfg"
            else
              echo "⚠️  zenoh_ope.json5 が見つかりません。"
            fi

            unset _repo_root _op_cfg

            echo "======================================================="
            echo " Rustacean RoboRescue — Operator Station"
            echo " ROS: ${ROS_VERSION} | RMW: rmw_zenoh_cpp"
            echo "======================================================="
          '';
        };
        # Lightweight devShell for running pre-commit and Python linters.
        # Use a plain nixpkgs import (pkgsPlain) to avoid overlay evaluation
        # issues from nixgl/ros overlays.
        devShells.precommit =
          let pkgsPlain = import nixpkgs { inherit system; };
          in pkgsPlain.mkShell {
            name = "RoboRescue Precommit Shell";
            packages = with pkgsPlain; [
              python3
              pre-commit          # top-level package
              python3Packages.ruff
              python3Packages.black
              python3Packages.isort
              # cpplint は language:python フックなので pre-commit が venv で管理
              # cppcheck / numpy は不要
              clang-tools         # clang-format (language:system フックが要求)
            ];
            shellHook = ''
              echo "Entering Pre-commit devShell: use 'pre-commit run --all-files'"
            '';
          };

        # CI 専用 devShell ─ GitHub Actions 上で使用する軽量シェル。
        # Gazebo / RViz2 / Nav2 など GUI・シミュレーション系を含まないため
        # Nix closure が小さく、magic-nix-cache / Cachix のヒット率が高い。
        devShells.ci = pkgs.mkShell {
          name = "RoboRescue CI";
          packages = [
            # --- Rust ---
            rustNightly
            pkgs.cargo-audit
            pkgs.cargo-tarpaulin

            # --- C++ ビルド ---
            pkgs.cmake pkgs.ninja pkgs.pkg-config pkgs.mold
            pkgs.clang pkgs.clang-tools pkgs.llvmPackages.libclang
            pkgs.boost pkgs.vtk pkgs.pcl
            pkgs.ccache
            pkgs.sccache

            # --- colcon / ROS ビルドインフラ ---
            pkgs.colcon
            pkgs.python3Packages.colcon-cargo
            pkgs.python3Packages.colcon-ros-cargo
            pkgs.cargo-ament-build
            pkgs.python3
            pkgs.python3Packages.numpy

            # --- GStreamer (audio_bridge) ---
            pkgs.gst_all_1.gstreamer
            pkgs.gst_all_1.gst-plugins-base
            pkgs.python3Packages.pygobject3

            # --- ROS 2 (軽量セット) ---
            ciEnv
          ];

          shellHook = ''
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2
            export RUST_SRC_PATH="${rustNightly}/lib/rustlib/src/rust/library"
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export TBBROOT="${tbbRuntime}"
            export CCACHE_DIR="''${XDG_CACHE_HOME:-$HOME/.cache}/ccache"
            export CCACHE_MAXSIZE="20G"
            export SCCACHE_DIR="''${XDG_CACHE_HOME:-$HOME/.cache}/sccache"
            export CC="ccache clang"
            export CXX="ccache clang++"
            export RUSTFLAGS="-C link-arg=-fuse-ld=mold"
            export CMAKE_PREFIX_PATH="${ciEnv}:$CMAKE_PREFIX_PATH"
            export AMENT_PREFIX_PATH="${ciEnv}:$AMENT_PREFIX_PATH"
            export LD_LIBRARY_PATH="${ciEnv}/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            export PYTHONPATH="${ciEnv}/${pkgs.python3.sitePackages}''${PYTHONPATH:+:$PYTHONPATH}"
          '';
        };
      }
    );
}
