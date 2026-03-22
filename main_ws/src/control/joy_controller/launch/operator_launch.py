from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="joy", executable="joy_node", name="pub_joy"),
            Node(
                package="joy_controller",
                executable="joy_controller_node",
                name="joy_controller",
                output="screen",
            ),
        ]
    )
