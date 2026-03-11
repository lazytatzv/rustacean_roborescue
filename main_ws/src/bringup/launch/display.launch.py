# ロボットモデル確認用 Launch
# robot_state_publisher + joint_state_publisher_gui + (optional) rviz2
#
# 使い方:
#   ros2 launch bringup display.launch.py                        # フルモデル + rviz2
#   ros2 launch bringup display.launch.py urdf:=sekirei.urdf     # アーム単体
#   ros2 launch bringup display.launch.py use_rviz:=false        # rviz なし (別途 nixGL rviz2 で起動)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")

    # ── Launch arguments ──
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value="robot.urdf.xacro",
        description="URDF/Xacro ファイル名 (bringup/urdf/ 内)",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="rviz2 を起動するか (Nix 環境では false にして nixGL rviz2 で別途起動)",
    )

    urdf_file = os.path.join(bringup_dir, "urdf")  # ディレクトリ部分
    # xacro は .urdf もそのまま通すので統一的に xacro 経由で処理
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file, "/", LaunchConfiguration("urdf")]),
        value_type=str,
    )

    # ── Nodes ──
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(bringup_dir, "rviz", "display.rviz")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        urdf_arg,
        use_rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
