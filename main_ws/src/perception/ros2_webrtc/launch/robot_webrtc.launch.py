from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    audio_device = LaunchConfiguration("audio_device", default="")
    bitrate = LaunchConfiguration("bitrate", default="64000")

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "audio_device", default_value="", description="Audio device for pulsesrc"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "bitrate", default_value="64000", description="Opus bitrate"
        )
    )

    robot = Node(
        package="ros2_webrtc",
        executable="robot_webrtc_node",
        name="robot_webrtc",
        output="screen",
        arguments=["--audio-device", audio_device, "--bitrate", bitrate],
    )

    ld.add_action(robot)
    return ld
