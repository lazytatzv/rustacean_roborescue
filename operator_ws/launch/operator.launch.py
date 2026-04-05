# Operator側起動スクリプト
import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_operator_actions(context):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    # operator_ws 直下を探す
    repo_root = os.path.dirname(this_dir)

    # 証明書の場所を特定
    project_cert = os.path.join(repo_root, "quic", "server.crt")
    # もし見つからなければ、指示された scp 先を想定
    if not os.path.isfile(project_cert):
        project_cert = "/home/yano/working/rustacean_roborescue/operator_ws/quic/server.crt"

    # Zenoh 設定を動的に生成 (相対パス問題を完全に回避)
    ope_cfg_content = f"""
    {{
      mode: "client",
      connect: {{
        endpoints: [
          "quic/100.114.200.30:7447",
          "tcp/100.114.200.30:7447",
          "quic/10.42.0.1:7447",
          "tcp/10.42.0.1:7447"
        ]
      }},
      scouting: {{ multicast: {{ enabled: false }} }},
      transport: {{
        shared_memory: {{ enabled: true }},
        link: {{
          tls: {{
            root_ca_certificate: "{project_cert}"
          }}
        }}
      }}
    }}
    """
    fd, dynamic_ope_cfg = tempfile.mkstemp(prefix="zenoh_ope_dynamic_", suffix=".json5")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(ope_cfg_content)

    return [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
        SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", dynamic_ope_cfg),
        SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
    ]


def generate_launch_description():
    use_audio = DeclareLaunchArgument("use_audio", default_value="true")
    use_joy = DeclareLaunchArgument("use_joy", default_value="true")
    use_foxglove = DeclareLaunchArgument("use_foxglove", default_value="true")
    ope_mic_device = DeclareLaunchArgument("ope_mic_device", default_value="")
    ope_spk_device = DeclareLaunchArgument("ope_spk_device", default_value="")
    bitrate = DeclareLaunchArgument("bitrate", default_value="32000")
    foxglove_address = DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0")
    foxglove_port = DeclareLaunchArgument("foxglove_port", default_value="8765")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=IfCondition(LaunchConfiguration("use_joy")),
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
        parameters=[
            {
                "port": LaunchConfiguration("foxglove_port"),
                "address": LaunchConfiguration("foxglove_address"),
            }
        ],
    )

    audio_sender = Node(
        package="audio_bridge",
        executable="audio_sender",
        name="operator_audio_sender",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[
            {
                "topic": "/operator/audio",
                "device": LaunchConfiguration("ope_mic_device"),
                "bitrate": LaunchConfiguration("bitrate"),
            }
        ],
    )

    audio_receiver = Node(
        package="audio_bridge",
        executable="audio_receiver",
        name="operator_audio_receiver",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[{"topic": "/robot/audio", "device": LaunchConfiguration("ope_spk_device")}],
    )

    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=_build_operator_actions))
    ld.add_action(use_audio)
    ld.add_action(use_joy)
    ld.add_action(use_foxglove)
    ld.add_action(ope_mic_device)
    ld.add_action(ope_spk_device)
    ld.add_action(bitrate)
    ld.add_action(foxglove_address)
    ld.add_action(foxglove_port)
    ld.add_action(joy_node)
    ld.add_action(foxglove_node)
    ld.add_action(audio_sender)
    ld.add_action(audio_receiver)

    return ld
