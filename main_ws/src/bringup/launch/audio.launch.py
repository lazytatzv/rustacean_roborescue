from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")
    signaling_launch = os.path.join(
        bringup_dir,
        "launch",
        "..",
        "..",
        "perception",
        "ros2_webrtc",
        "launch",
        "signaling.launch.py",
    )
    # fallback: use package ros2_webrtc if installed
    signaling_launch = (
        os.path.join(
            get_package_share_directory("ros2_webrtc"), "launch", "signaling.launch.py"
        )
        if os.path.exists(
            os.path.join(
                get_package_share_directory("ros2_webrtc"),
                "launch",
                "signaling.launch.py",
            )
        )
        else signaling_launch
    )

    audio_device = DeclareLaunchArgument(
        "audio_device", default_value="", description="Audio device for robot"
    )
    bitrate = DeclareLaunchArgument(
        "bitrate", default_value="64000", description="Opus bitrate"
    )

    ld = LaunchDescription()
    ld.add_action(audio_device)
    ld.add_action(bitrate)

    # Start signaling (if provided by ros2_webrtc package)
    try:
        signaling = Node(
            package="ros2_webrtc",
            executable="signaling_node",
            name="webrtc_signaling",
            output="screen",
        )
        ld.add_action(signaling)
    except Exception:
        pass

    # Start robot webrtc node
    robot = Node(
        package="ros2_webrtc",
        executable="robot_webrtc_node",
        name="robot_webrtc",
        output="screen",
        arguments=[
            "--audio-device",
            LaunchConfiguration("audio_device"),
            "--bitrate",
            LaunchConfiguration("bitrate"),
        ],
    )

    ld.add_action(robot)

    return ld
