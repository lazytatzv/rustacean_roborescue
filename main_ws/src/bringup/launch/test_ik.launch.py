"""IK 動作確認用 Launch ファイル.

実機なしで arm_controller の IK を RViz 上で確認する。

起動後、別ターミナルから Twist を送ると EE が動く:
  ros2 topic pub /arm_cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.01}}" --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")

    arm_params = DeclareLaunchArgument(
        "arm_params",
        default_value=os.path.join(bringup_dir, "config", "arm_controller.yaml"),
    )

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(bringup_dir, "urdf", "robot.urdf.xacro")]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    arm_controller = Node(
        package="arm_controller",
        executable="arm_controller",
        name="arm_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("arm_params"),
            {"robot_description": robot_description},
        ],
    )

    arm_mock_driver = Node(
        package="bringup",
        executable="arm_mock_driver.py",
        name="arm_mock_driver",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(bringup_dir, "rviz", "display.rviz")],
    )

    return LaunchDescription(
        [
            arm_params,
            robot_state_publisher,
            arm_mock_driver,
            arm_controller,
            rviz,
        ]
    )
