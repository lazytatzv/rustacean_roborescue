# Operator側起動スクリプト
# 操縦側で立ち上げるのはこれだけで完結させること

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Set RMW implementation and Zenoh config URI for all nodes launched here
    # Compute zenoh config path relative to this file: ../zenoh_ope.json5
    this_dir = os.path.dirname(__file__)
    repo_root = os.path.dirname(this_dir)
    zenoh_config_path = os.path.join(repo_root, "zenoh_ope.json5")
    zenoh_config_uri = f"file://{zenoh_config_path}"

    set_rmw = SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    set_ros_domain = SetEnvironmentVariable("ROS_DOMAIN_ID", "0")
    set_ros_localhost_only = SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0")
    # Keep waiting for router instead of failing the initial one-shot check.
    set_router_check_attempts = SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1")
    set_zenoh_uri = SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", zenoh_config_uri)
    # NOTE: In this environment, rmw_zenoh_cpp can ignore RMW_ZENOH_CONFIG_URI and fall back
    # to tcp/localhost:7447. Force the remote router endpoint explicitly.
    set_zenoh_override = SetEnvironmentVariable(
        "ZENOH_CONFIG_OVERRIDE",
        'mode="client";connect/endpoints=["tcp/10.42.0.1:7447"];scouting/multicast/enabled=false',
    )

    use_audio = DeclareLaunchArgument(
        "use_audio", default_value="true", description="音声ノードを起動するか"
    )
    use_joy = DeclareLaunchArgument(
        "use_joy", default_value="true", description="joy_node を起動するか"
    )
    use_foxglove = DeclareLaunchArgument(
        "use_foxglove", default_value="true", description="foxglove_bridge を起動するか"
    )
    ope_mic_device = DeclareLaunchArgument(
        "ope_mic_device",
        default_value="",
        description="オペレータ マイクの PulseAudio デバイス名 (空=デフォルト)",
    )
    ope_spk_device = DeclareLaunchArgument(
        "ope_spk_device",
        default_value="",
        description="オペレータ スピーカーの PulseAudio デバイス名 (空=デフォルト)",
    )
    bitrate = DeclareLaunchArgument(
        "bitrate", default_value="32000", description="Opus ビットレート [bps]"
    )
    foxglove_address = DeclareLaunchArgument(
        "foxglove_address",
        default_value="0.0.0.0",
        description="foxglove_bridge bind address",
    )
    foxglove_port = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="foxglove_bridge WebSocket port",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_joy")),
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
        parameters=[
            {
                "port": LaunchConfiguration("foxglove_port"),
                "address": LaunchConfiguration("foxglove_address"),
                "send_buffer_limit": 10000000,
                "max_qos_depth": 10,
                "num_threads": 0,
                "use_compression": False,
            }
        ],
    )

    # マイク → /operator/audio (Zenoh 経由で NUC へ)
    audio_sender = Node(
        package="audio_bridge",
        executable="audio_sender",
        name="operator_audio_sender",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[
            {
                "topic": "/operator/audio",
                "device": LaunchConfiguration("ope_mic_device"),
                "bitrate": LaunchConfiguration("bitrate"),
            }
        ],
    )

    # /robot/audio → スピーカー
    audio_receiver = Node(
        package="audio_bridge",
        executable="audio_receiver",
        name="operator_audio_receiver",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[
            {
                "topic": "/robot/audio",
                "device": LaunchConfiguration("ope_spk_device"),
            }
        ],
    )

    ld = LaunchDescription()
    # environment first
    ld.add_action(set_rmw)
    ld.add_action(set_ros_domain)
    ld.add_action(set_ros_localhost_only)
    ld.add_action(set_router_check_attempts)
    ld.add_action(set_zenoh_uri)
    ld.add_action(set_zenoh_override)
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
