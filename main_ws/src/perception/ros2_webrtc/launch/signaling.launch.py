from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port", default="8080")

    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "port", default_value="8080", description="Signaling port"
        )
    )

    signaling = Node(
        package="ros2_webrtc",
        executable="signaling_node",
        name="webrtc_signaling",
        output="screen",
        parameters=[
            {
                "SIGNALING_PORT": port,
            }
        ],
    )

    ld.add_action(signaling)
    return ld
