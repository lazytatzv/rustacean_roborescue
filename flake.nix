{
  description = "Pro Rustacean RoboRescue Env";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, rust-overlay, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default (import rust-overlay) ];
        };

        ROS_VERSION = "jazzy";
        
        rustNightly = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" ];
          targets = [ "thumbv7em-none-eabihf" ];
        };

        # 共通のパッケージリスト（ここを編集すれば両方に反映される）
        basePackages = with pkgs; [
          # --- 開発ツール ---
          clang llvmPackages.clang pkg-config git rustNightly colcon
          fish which procps  # 👈 これらが Docker 内の快適さを決める
          # --- ROS 2 ---
          rosPackages.${ROS_VERSION}.ros-core
          rosPackages.${ROS_VERSION}.ros-environment
          rosPackages.${ROS_VERSION}.ament-cmake
          rosPackages.${ROS_VERSION}.rosidl-generator-rs 
          rosPackages.${ROS_VERSION}.std-msgs
          rosPackages.${ROS_VERSION}.sensor-msgs
          rosPackages.${ROS_VERSION}.geometry-msgs
          rosPackages.${ROS_VERSION}.builtin-interfaces
          # --- Python ---
          python3Packages.colcon-cargo
          python3Packages.colcon-ros-cargo
          python3Packages.empy
          python3Packages.lark
          python3Packages.numpy
        ];

        # Docker用にパッケージを一つのディレクトリ構造にまとめる魔法
        envApp = pkgs.buildEnv {
          name = "robo-env-root";
          paths = basePackages;
          pathsToLink = [ "/bin" "/lib" "/share" "/include" ];
        };

      in
      {
        # 1. 開発環境（いつものやつ）
        devShells.default = pkgs.mkShell {
          name = "RoboRescue Pro Env";
          packages = basePackages;
          shellHook = ''
            export DIRENV_LOG_FORMAT=""
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export RUST_SRC_PATH="${rustNightly}/lib/rustlib/src/rust/library"
            export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${pkgs.lib.makeLibraryPath basePackages}"
            echo "=========================================="
            echo "🦀 PRO Environment Loaded (Jazzy + Rust) 🦀"
            echo "=========================================="
          '';
        };

        # 2. 配布用Dockerイメージ（最強版）
        packages.dockerImage = pkgs.dockerTools.buildLayeredImage {
          name = "lazytatzv/robo-env";
          tag = "latest";
          
          # buildEnv でまとめた中身 + bash を入れる
          contents = [ envApp pkgs.bashInteractive pkgs.coreutils ];

          config = {
            Cmd = [ "/bin/bash" ]; # 起動時はとりあえずbash
            Env = [
              "PATH=/bin" # 👈 これで /bin/fish や /bin/ros2 が見つかる！
              "ROS_DISTRO=${ROS_VERSION}"
              "ROS_VERSION=2"
              "LD_LIBRARY_PATH=/lib"
              "PYTHONPATH=/lib/python3.11/site-packages" # パスは環境に合わせて調整
              "AMENT_PREFIX_PATH=/bin" # これがないと ROS コマンドが死ぬ
            ];
          };
        };
      }
    );
}
