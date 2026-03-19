{
  description = "Rustacean RoboRescue — オペレータ PC 最小環境 (joy_node + Zenoh)";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        ROS_VERSION = "jazzy";
        ros = pkgs.rosPackages.${ROS_VERSION};

        operatorDeps = [
          ros.ros-core           # rclcpp, rmw, etc.
          ros.rmw-zenoh-cpp      # Zenoh RMW implementation
          ros.joy                # joy_node (PS4 / Xbox コントローラ)
          ros.joy-linux          # Linux joystick backend
          ros.foxglove-bridge    # Foxglove Bridge for visualization
          ros.ros2cli-common-extensions
        ];

        operatorEnv = pkgs.symlinkJoin {
          name = "roborescue-operator-env";
          paths = operatorDeps;
        };

      in {
        devShells.default = pkgs.mkShell {
          name = "RoboRescue Operator";

          packages = [ operatorEnv pkgs.zenoh ];

          shellHook = ''
            export ROS_DISTRO="${ROS_VERSION}"
            export ROS_VERSION=2
            export RMW_IMPLEMENTATION=rmw_zenoh_cpp
            export ZENOH_ROUTER_CHECK_ATTEMPTS=-1

            # ── Zenoh 設定 ──
            # <ROBOT_IP> を実機 IP に書き換えた zenoh_ope.json5 を指定
            ZENOH_CFG="$PWD/zenoh_ope.json5"
            if [ -f "$ZENOH_CFG" ]; then
              export RMW_ZENOH_CONFIG_URI="file://$ZENOH_CFG"
              echo "✅ Zenoh config: $ZENOH_CFG"
            else
              echo "⚠️  zenoh_ope.json5 が見つかりません。以下を作成してください:"
              echo "   cp ../main_ws/src/bringup/config/zenoh_ope.json5 ."
              echo "   # <ROBOT_IP> をロボットの IP アドレスに書き換える"
            fi

            echo "======================================================="
            echo " Rustacean RoboRescue — Operator Station"
            echo " ROS: ${ROS_VERSION} | RMW: rmw_zenoh_cpp"
            echo ""
            echo " 使い方:"
            echo "   1. PS4 コントローラを USB/Bluetooth 接続"
            echo "   2. ros2 run joy joy_node"
            echo "   3. Foxglove Studio → ws://<ROBOT_IP>:8765"
            echo "======================================================="
          '';
        };
      }
    );
}
