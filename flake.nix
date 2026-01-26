{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        rosDistro = pkgs.rosPackages.jazzy; # Jazzyを指定
      in {
        devShells.default = pkgs.mkShell {
          name = "Rust ROS 2 Jazzy Env";
          
          # ビルドに必要なツール群
          nativeBuildInputs = with pkgs; [
            cmake
            clang
            pkg-config
            colcon
            cargo-ament-build
            python3Packages.colcon-ros-cargo
            python3Packages.colcon-cargo
            git
            # Rust
            cargo
            rustc
            # Python (メッセージ生成ツールが依存)
            (python3.withPackages (ps: with ps; [ empy lark setuptools ]))
          ];

          # リンクするライブラリ (Nixのバイナリを使うのでビルド不要！)
          buildInputs = with rosDistro; [
            ros-core
            ament-cmake
            rosidl-default-generators
            std-msgs        # C++用ヘッダ
            sensor-msgs     # C++用ヘッダ
            geometry-msgs
            # rclrsが依存するCライブラリ
            pkgs.libyaml
          ];

          shellHook = ''
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            # C++ヘッダをBindgenが見つけられるようにする
            export BINDGEN_EXTRA_CLANG_ARGS="$(< ${pkgs.stdenv.cc}/nix-support/libc-cflags) \
              -idirafter ${rosDistro.ros-core}/include \
              -idirafter ${rosDistro.std-msgs}/include \
              -idirafter ${rosDistro.sensor-msgs}/include"
            
            # NixのCMakeパスをcolconに教える
            export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$AMENT_PREFIX_PATH
            
            echo "🦀 RCLRS Jazzy Hybrid Environment Loaded"
          '';
        };
      });
}
