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
      in {
        devShells.default = pkgs.mkShell {
          name = "OpenCV WeChat QR Project";
          
          nativeBuildInputs = with pkgs; [
            pkg-config
            # Provides libclang.so for bindgen
            llvmPackages.libclang.lib
            clang
          ];

          buildInputs = with pkgs; [
            # Your standard ROS packages
            (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [
                #ros-core
                desktop
                cv-bridge
                # Add other ROS dependencies here
              ];
            })
            opencv
          ];

          shellHook = ''
            export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"
            export PKG_CONFIG_PATH="$PKG_CONFIG_PATH:${pkgs.opencv}/lib/pkgconfig"
            
            # Fix for QT/GUI apps if they don't launch
            export QT_QPA_PLATFORM=xcb
          '';


        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
