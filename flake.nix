{
  description = "Pro Rustacean RoboRescue Env";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    
    # 👇 キャッシュヒット率向上のための設定
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
          extensions = [ "rust-src" "rust-analyzer" ];
        };

        # 🛠️ 【修正】変数の定義はここ (letの中) に書きます
        myRosPackages = with pkgs.rosPackages.${ROS_VERSION}; [
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

            # デモ用パッケージ
            demo-nodes-cpp
            demo-nodes-py
            joy
        ];

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
            rustNightly
            
            # Colcon本体
            colcon
          ]) 
          # 🤖 ROS 2 パッケージ (上で定義した変数をここで足す)
          ++ myRosPackages
          # 🐍 Pythonツール
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

            # 🛠️ 【重要】ライブラリパスをNixの魔法で自動生成
            # これで demo_nodes_cpp が動くようになります
            export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${pkgs.lib.makeLibraryPath myRosPackages}"
            
            # 常にRust生成をON
            export ROSIDL_GENERATOR_RUST=ON
            
            # ローカルのワークスペース設定読み込み
            if [ -f install/setup.bash ]; then
              source install/setup.bash
            fi

            echo "=========================================="
            echo "🦀 PRO Environment Loaded (Jazzy + Rust) 🦀"
            echo "   - demo-nodes-cpp installed"
            echo "=========================================="
          '';
        };
      }
    );
}
