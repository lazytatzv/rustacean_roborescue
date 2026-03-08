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
          config.allowUnfree = true;
        };

        ROS_VERSION = "jazzy";

        rustNightly = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "llvm-tools-preview" ];
        };

        # 🛠️ ツール類（パス爆発の原因にならないもの）
        buildTools = with pkgs; [
          cmake ccache mold ninja clang-tools clang llvmPackages.openmp
          fish fishPlugins.bass just pkgs.nixgl.auto.nixGLDefault
          rustNightly bacon cargo-watch cargo-expand cargo-binutils cargo-ament-build probe-rs zenoh
          python3 python3Packages.numpy python3Packages.opencv4 python3Packages.black python3Packages.isort python3Packages.ipdb
          colcon python3Packages.colcon-cargo python3Packages.colcon-ros-cargo
          git lazygit ripgrep fd btop zellij tmux nodePackages.mermaid-cli boost

          rosPackages.${ROS_VERSION}.ros-core
          rosPackages.${ROS_VERSION}.ament-cmake
        ];

        # 📦 ROS 2 パッケージ群（これらがパス爆発の真犯人）
        rosDeps = with pkgs; [
          eigen vtk
          #rosPackages.${ROS_VERSION}.ros-core
          #rosPackages.${ROS_VERSION}.ament-cmake
          rosPackages.${ROS_VERSION}.std-msgs
          rosPackages.${ROS_VERSION}.sensor-msgs
          rosPackages.${ROS_VERSION}.geometry-msgs
          rosPackages.${ROS_VERSION}.nav-msgs
          rosPackages.${ROS_VERSION}.visualization-msgs
          rosPackages.${ROS_VERSION}.rviz2
          rosPackages.${ROS_VERSION}.rqt-graph
          rosPackages.${ROS_VERSION}.rqt-plot
          rosPackages.${ROS_VERSION}.rqt-console
          rosPackages.${ROS_VERSION}.joy
          rosPackages.${ROS_VERSION}.joy-linux
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
          rosPackages.${ROS_VERSION}.dynamixel-workbench-toolbox
          rosPackages.${ROS_VERSION}.dynamixel-sdk
        ];

        # 🌟 最終奥義：ROSライブラリだけを1つのディレクトリに完全圧縮
        roboRescueEnv = pkgs.symlinkJoin {
          name = "roborescue-compressed-env";
          paths = rosDeps;
        };

      in
      {
        devShells.default = pkgs.mkShell {
          name = "RoboRescue Env";
          
          # 🚨 超重要：rosDeps をシェルに直接渡さない！ 圧縮した環境だけを渡す！
          packages = buildTools ++ [ roboRescueEnv ];

          shellHook = ''
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2
            
            export RUST_SRC_PATH="${rustNightly}/lib/rustlib/src/rust/library"
            
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export CC="ccache clang"
            export CXX="ccache clang++"
            export RUSTFLAGS="-C link-arg=-fuse-ld=mold"
            
            export CFLAGS="-fopenmp $CFLAGS"
            export CXXFLAGS="-fopenmp $CXXFLAGS"
            export LDFLAGS="-fopenmp $LDFLAGS"

            alias ros2="nixGL ros2"
            alias rviz2="nixGL rviz2"
            alias rqt="nixGL rqt"
            alias rqt_graph="nixGL rqt_graph"
            
            # ライブラリパスを圧縮ディレクトリ1つだけに指定
            export LD_LIBRARY_PATH="${roboRescueEnv}/lib:$LD_LIBRARY_PATH"
            export CMAKE_PREFIX_PATH="${roboRescueEnv}:${pkgs.vtk}/lib/cmake/vtk:$CMAKE_PREFIX_PATH"
            export AMENT_PREFIX_PATH="${roboRescueEnv}:$AMENT_PREFIX_PATH"

            echo "======================================================="
            echo " Ready to Dev ! (完全圧縮モード発動 🚀)"
            echo "======================================================="
          '';
        };
      }
    );
}
