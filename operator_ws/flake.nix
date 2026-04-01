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
          ros.rclpy
          ros.rosidl-default-generators
          ros.rosidl-default-runtime
          ros.image-transport
          ros.ffmpeg-image-transport       # /camera/image_raw/ffmpeg の型定義
          ros.ffmpeg-image-transport-msgs  # FFMPEGPacket メッセージ型
          # audio_bridge 依存: GStreamer + Python GI bindings
          pkgs.gst_all_1.gstreamer
          pkgs.gst_all_1.gst-plugins-base  # opusdec, audioconvert, audioresample
          pkgs.gst_all_1.gst-plugins-good  # pulsesrc, pulsesink
          pkgs.gobject-introspection
          pkgs.python3Packages.pygobject3
          pkgs.pulseaudio
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
            export LD_LIBRARY_PATH="${ros.rmw-zenoh-cpp}/lib:${ros.rmw-zenoh-cpp}/lib64:${ros.zenoh-cpp-vendor}/lib:${ros.zenoh-cpp-vendor}/lib64''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

            # ── Zenoh 設定 ──
            ZENOH_CFG="$PWD/zenoh_ope.json5"
            if [ -f "$ZENOH_CFG" ]; then
              export RMW_ZENOH_CONFIG_URI="file://$ZENOH_CFG"
              echo "✅ Zenoh config: $ZENOH_CFG"
            else
              echo "⚠️  zenoh_ope.json5 が見つかりません。以下を作成してください:"
              echo "   cp ../operator/zenoh_ope.json5 ."
            fi

            echo "======================================================="
            echo " Rustacean RoboRescue — Operator Station"
            echo " ROS: ${ROS_VERSION} | RMW: rmw_zenoh_cpp"
            echo "======================================================="
          '';
        };
      }
    );
}
