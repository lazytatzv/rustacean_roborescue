{
  description = "robot dev env";

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
            nix-ros-overlay.overlays.default (import rust-overlay)
            nixgl.overlay
          ];
          config.allowUnfree = true; # VSCodeやプロプライエタリなツール用
        };

        ROS_VERSION = "jazzy";

        # Rust環境seutp
        rustNightly = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "llvm-tools-preview" ];
          targets = [ "thumbv7em-none-eabihf" ]; # stm32用だが、基本的に必要ないかも
        };

        basePackages = with pkgs; [
          cmake
          eigen
          vtk
          # =========================================
          #  dev tools
          # =========================================
          ccache          # C++の再ビルドを爆速にするキャッシュ
          mold            # 現代の最強リンカ (ld/lldより圧倒的に速い)
          ninja           # Makeより速いビルドシステム
          clang-tools     # clang-format (C++の整形)
          clang
          llvmPackages.openmp  # OpenMP (omp.h)

          # fishまわり
          fish
          fishPlugins.bass

          just

          # nixgl設定
          pkgs.nixgl.auto.nixGLDefault
          
          # =========================================
          #  rust optimization tools
          # =========================================
          rustNightly
          bacon           # バックグラウンドで常にコンパイルエラーを監視してくれる神ツール
          cargo-watch     # ファイル変更検知して自動コマンド実行
          cargo-expand    # マクロ展開後のコードを見る (ROS2マクロのデバッグに便利)
          cargo-binutils  # Embedded用 (objcopy, size)
          probe-rs        # STM32への書き込み・デバッグ (OpenOCDより楽)

          zenoh
          # =========================================
          #  Python & Vision ツール
          # =========================================
          python3
          python3Packages.numpy
          python3Packages.opencv4
          python3Packages.black   # Pythonコード整形
          python3Packages.isort   # import順序整形
          python3Packages.ipdb    # インタラクティブデバッガ (printデバッグ卒業)
          
          # =========================================
          #  ROS 2 Utilities
          # =========================================
          colcon
          #rosPackages.${ROS_VERSION}.colcon-common-extensions
          python3Packages.colcon-cargo
          python3Packages.colcon-ros-cargo
          rosPackages.${ROS_VERSION}.ros-core
          rosPackages.${ROS_VERSION}.ament-cmake
          rosPackages.${ROS_VERSION}.rosidl-generator-rs 
          
          # 必須メッセージ型
          rosPackages.${ROS_VERSION}.std-msgs
          rosPackages.${ROS_VERSION}.sensor-msgs
          rosPackages.${ROS_VERSION}.geometry-msgs
          rosPackages.${ROS_VERSION}.nav-msgs
          rosPackages.${ROS_VERSION}.visualization-msgs # Rviz用

          # 可視化・GUIツール (これがないとデバッグできない)
          rosPackages.${ROS_VERSION}.rviz2      # 3D可視化
          rosPackages.${ROS_VERSION}.rqt-graph  # ノードの接続グラフを見る
          rosPackages.${ROS_VERSION}.rqt-plot   # センサー値のグラフ化
          rosPackages.${ROS_VERSION}.rqt-console # ログを見る

          # いろいろ便利なやつ
          rosPackages.${ROS_VERSION}.joy
          rosPackages.${ROS_VERSION}.joy-linux # linux用のドライバ
          rosPackages.${ROS_VERSION}.teleop-twist-joy
          rosPackages.${ROS_VERSION}.demo-nodes-cpp
          rosPackages.${ROS_VERSION}.demo-nodes-py
          rosPackages.${ROS_VERSION}.image-tools
          rosPackages.${ROS_VERSION}.image-transport
          rosPackages.${ROS_VERSION}.compressed-image-transport
          rosPackages.${ROS_VERSION}.velodyne
          rosPackages.${ROS_VERSION}.usb-cam
          rosPackages.${ROS_VERSION}.pcl-ros
          rosPackages.${ROS_VERSION}.pcl-conversions
          rosPackages.${ROS_VERSION}.tf2-eigen
          rosPackages.${ROS_VERSION}.tf2-sensor-msgs
          rosPackages.${ROS_VERSION}.pointcloud-to-laserscan
          rosPackages.${ROS_VERSION}.slam-toolbox

          # =========================================
          #  ターミナル & システム管理
          # =========================================
          git
          lazygit         
          ripgrep         
          fd              
          btop            
          zellij          
          tmux

          nodePackages.mermaid-cli
        ];

      in
      {
        devShells.default = pkgs.mkShell {
          name = "RoboRescue Env";
          packages = basePackages;

          shellHook = ''
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2
            
            # --- Rust設定 ---
            export RUST_SRC_PATH="${rustNightly}/lib/rustlib/src/rust/library"
            
            # --- C++ & ビルド高速化設定 ---
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export CC="ccache clang"   # ccacheを噛ませて高速化
            export CXX="ccache clang++"
            # リンカをmoldに強制し、高速化
            export RUSTFLAGS="-C link-arg=-fuse-ld=mold"
            
            # OpenMP 
            # fast-lio用に必要
            export CFLAGS="-fopenmp $CFLAGS"
            export CXXFLAGS="-fopenmp $CXXFLAGS"
            export LDFLAGS="-fopenmp $LDFLAGS"

            # GUIを使うならnixGL経由であることが必要
            alias ros2="nixGL ros2"
            alias rviz2="nixGL rviz2"
            alias rqt="nixGL rqt"
            alais rqt_graph="nixGL rqt_graph"
            
            # zenoh設定
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            # zenohを黙らせる
            #export RUST_LOG=warn,zenoh=error

            # --- ライブラリパス ---
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath basePackages}:$LD_LIBRARY_PATH"
            
            # --- CMake設定 (VTK等を認識させる) ---
            export CMAKE_PREFIX_PATH="${pkgs.vtk}/lib/cmake/vtk:$CMAKE_PREFIX_PATH"

            echo "======================================================="
            echo " Ready to Dev !"
            echo "======================================================="
          '';
        };
      }
    );
}
