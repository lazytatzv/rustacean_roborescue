{
  description = "Pro Rustacean RoboRescue Env";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    
    # 👇 ここを修正しました！
    # url を削除し、follows だけにすることで、ROS overlayが使っているnixpkgsを強制的に使わせます。
    # これにより、バイナリキャッシュがヒットしやすくなり、ビルド時間が激減します。
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nix-ros-overlay, nixpkgs, rust-overlay }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            nix-ros-overlay.overlays.default
            (import rust-overlay)
            ];
        };
        ROS_VERSION = "jazzy";
        
        rustNightly = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" ]; # 補完に必要なソースも同梱
        };

      in
      {
        devShells.default = pkgs.mkShell {
          name = "RoboRescue Pro Env";
          
          # 📦 必要なツールとライブラリ
          packages = (with pkgs; [
            # ビルドツール & 言語
            clang
            llvmPackages.clang
            pkg-config
            git
            #rustup
            rustNightly
            
            # Colcon本体
            colcon
          ]) 
          # 🤖 ROS 2 パッケージ (Jazzy)
          ++ (with pkgs.rosPackages.${ROS_VERSION}; [
            ros-core
            ros-environment
            ament-cmake
            
            # 職人を完成品として入れる
            rosidl-generator-rs 
            
            # メッセージ定義
            std-msgs
            sensor-msgs
            geometry-msgs
            builtin-interfaces
          ])
          # 🐍 Pythonツール (Colconプラグイン & 生成ツール)
          ++ (with pkgs.python3Packages; [
            colcon-cargo
            colcon-ros-cargo
            
            empy
            lark
            numpy
          ]);

          # 🔧 環境変数の設定
          shellHook = ''
            # ROS環境のロード
            source ${pkgs.rosPackages.${ROS_VERSION}.ros-environment}/setup.bash
            
            # Bindgen用パス設定
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export BINDGEN_EXTRA_CLANG_ARGS="$(< ${pkgs.stdenv.cc}/nix-support/libc-cflags) \
              -idirafter ${pkgs.rosPackages.${ROS_VERSION}.ros-core}/include \
              -idirafter ${pkgs.rosPackages.${ROS_VERSION}.sensor-msgs}/include \
              -idirafter ${pkgs.rosPackages.${ROS_VERSION}.std-msgs}/include"

            # Rustupの設定
            export RUST_SRC_PATH="${rustNightly}/lib/rustlib/src/rust/library"

            # まだRustが入ってなければ入れる
            if ! command -v rustc &> /dev/null; then
                echo "🦀 Installing Rust Nightly..."
                rustup toolchain install nightly
                rustup default nightly
            fi
            
            # 常にRust生成をON
            export ROSIDL_GENERATOR_RUST=ON
            
            # ローカルのワークスペース設定読み込み
            if [ -f install/setup.bash ]; then
              source install/setup.bash
            fi

            echo "=========================================="
            echo "🦀 PRO Environment Loaded (Jazzy + Rust) 🦀"
            echo "=========================================="
          '';
        };
      }
    );
}
