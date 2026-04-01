import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("bringup")

    # ── Launch arguments ──
    arm_params = DeclareLaunchArgument(
        "arm_params",
        default_value=os.path.join(bringup_share, "config", "arm_controller.yaml"),
    )
    arm_driver_params = DeclareLaunchArgument(
        "arm_driver_params",
        default_value=os.path.join(bringup_share, "config", "arm_gripper_driver.yaml"),
    )
    joy_params = DeclareLaunchArgument(
        "joy_params",
        default_value=os.path.join(bringup_share, "config", "joy_controller.yaml"),
    )

    # ── Shared respawn config ──
    respawn_config = {"respawn": True, "respawn_delay": 2.0}

    # ── Nodes ──
    # 1. Joy Node (Standard ROS 2 node to read /dev/input/jsX)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }
        ],
    )

    # 2. Joy Controller (Our node to map /joy to /arm_cmd_vel)
    joy_controller_node = Node(
        package="joy_controller",
        executable="joy_controller_node",
        name="joy_controller",
        output="screen",
        parameters=[LaunchConfiguration("joy_params")],
        **respawn_config,
    )

    # 3. Arm Driver (Hardware communication)
    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver",
        name="arm_driver",
        output="screen",
        parameters=[LaunchConfiguration("arm_driver_params")],
        **respawn_config,
    )

    # 4. Arm Controller (Inverse Kinematics)
    # Resolve absolute path for URDF
    sekirei_urdf = os.path.join(bringup_share, "urdf", "sekirei.urdf")

    arm_controller_node = Node(
        package="arm_controller",
        executable="arm_controller",
        name="arm_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("arm_params"),
            {"urdf_path": sekirei_urdf},
        ],
        **respawn_config,
    )

    return LaunchDescription(
        [
            arm_params,
            arm_driver_params,
            joy_params,
            joy_node,
            joy_controller_node,
            arm_driver_node,
            arm_controller_node,
        ]
    )
