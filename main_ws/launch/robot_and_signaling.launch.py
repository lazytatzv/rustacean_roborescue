from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    signaling_port = LaunchConfiguration("signaling_port", default="8080")
    audio_device = LaunchConfiguration("audio_device", default="")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("signaling_port", default_value="8080"))
    ld.add_action(DeclareLaunchArgument("audio_device", default_value=""))

    signaling = Node(
        package="ros2_webrtc",
        executable="signaling_node",
        name="webrtc_signaling",
        output="screen",
        parameters=[{"SIGNALING_PORT": signaling_port}],
    )

    robot = Node(
        package="ros2_webrtc",
        executable="robot_webrtc_node",
        name="robot_webrtc",
        output="screen",
        arguments=["--audio-device", audio_device],
    )

    ld.add_action(signaling)
    ld.add_action(robot)
    return ld
