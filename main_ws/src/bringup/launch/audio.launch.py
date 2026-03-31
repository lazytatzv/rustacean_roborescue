# NUC 側の双方向音声ノード
#
# audio_sender:   マイク → Opus → /robot/audio  (Zenoh 経由でオペレータへ)
# audio_receiver: /operator/audio  → Opus decode → スピーカー

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_audio = DeclareLaunchArgument(
        "use_audio", default_value="true", description="音声ノードを起動するか"
    )
    robot_device = DeclareLaunchArgument(
        "robot_mic_device",
        default_value="",
        description="NUC マイクの PulseAudio デバイス名 (空=デフォルト)",
    )
    robot_spk_device = DeclareLaunchArgument(
        "robot_spk_device",
        default_value="",
        description="NUC スピーカーの PulseAudio デバイス名 (空=デフォルト)",
    )
    bitrate = DeclareLaunchArgument(
        "bitrate", default_value="32000", description="Opus ビットレート [bps]"
    )

    # マイク → /robot/audio
    sender = Node(
        package="audio_bridge",
        executable="audio_sender",
        name="robot_audio_sender",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[
            {
                "topic": "/robot/audio",
                "device": LaunchConfiguration("robot_mic_device"),
                "bitrate": LaunchConfiguration("bitrate"),
            }
        ],
    )

    # /operator/audio → スピーカー
    receiver = Node(
        package="audio_bridge",
        executable="audio_receiver",
        name="robot_audio_receiver",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_audio")),
        parameters=[
            {
                "topic": "/operator/audio",
                "device": LaunchConfiguration("robot_spk_device"),
            }
        ],
    )

    return LaunchDescription([use_audio, robot_device, robot_spk_device, bitrate, sender, receiver])
