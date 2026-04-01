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
    set_zenoh_uri = SetEnvironmentVariable("RMW_ZENOH_CONFIG_URI", zenoh_config_uri)

    use_audio = DeclareLaunchArgument(
        "use_audio", default_value="true", description="音声ノードを起動するか"
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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "port": 8765,
                "address": "127.0.0.1",
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
    ld.add_action(set_zenoh_uri)
    ld.add_action(use_audio)
    ld.add_action(ope_mic_device)
    ld.add_action(ope_spk_device)
    ld.add_action(bitrate)
    ld.add_action(joy_node)
    ld.add_action(foxglove_node)
    ld.add_action(audio_sender)
    ld.add_action(audio_receiver)

    return ld
