from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")

    # ── Launch arguments ──
    crawler_params = DeclareLaunchArgument(
        "crawler_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "crawler_driver.yaml"]),
    )
    flipper_params = DeclareLaunchArgument(
        "flipper_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "flipper_driver.yaml"]),
    )
    arm_params = DeclareLaunchArgument(
        "arm_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "arm_controller.yaml"]),
    )
    sensor_gw_params = DeclareLaunchArgument(
        "sensor_gw_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "sensor_gateway.yaml"]),
    )
    gripper_params = DeclareLaunchArgument(
        "gripper_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "gripper_driver.yaml"]),
    )
    arm_driver_params = DeclareLaunchArgument(
        "arm_driver_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "arm_driver.yaml"]),
    )
    joy_params = DeclareLaunchArgument(
        "joy_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "joy_controller.yaml"]),
    )

    # ── Shared respawn config for hardware drivers ──
    # Hardware drivers automatically restart on crash (e.g., USB disconnect).
    # respawn_delay prevents rapid restart loops.
    hw_respawn = {"respawn": True, "respawn_delay": 3.0}

    # ── Hardware driver nodes (with respawn) ──
    crawler_driver_node = Node(
        package="crawler_driver",
        executable="crawler_driver_node",
        name="crawler_driver",
        output="both",
        parameters=[LaunchConfiguration("crawler_params")],
        **hw_respawn,
    )

    flipper_driver_node = Node(
        package="flipper_driver",
        executable="flipper_driver_node",
        name="flipper_driver",
        output="both",
        parameters=[LaunchConfiguration("flipper_params")],
        **hw_respawn,
    )

    sensor_gateway_node = Node(
        package="sensor_gateway",
        executable="sensor_gateway",
        name="sensor_gateway",
        output="both",
        parameters=[LaunchConfiguration("sensor_gw_params")],
        **hw_respawn,
    )

    gripper_driver_node = Node(
        package="gripper_driver",
        executable="gripper_driver",
        name="gripper_driver",
        output="both",
        parameters=[LaunchConfiguration("gripper_params")],
        **hw_respawn,
    )

    arm_driver_node = Node(
        package="arm_driver",
        executable="arm_driver",
        name="arm_driver",
        output="both",
        parameters=[LaunchConfiguration("arm_driver_params")],
        **hw_respawn,
    )

    # ── Control nodes (with respawn) ──
    ctrl_respawn = {"respawn": True, "respawn_delay": 2.0}

    joy_controller_node = Node(
        package="joy_controller",
        executable="joy_controller_node",
        name="joy_controller",
        output="both",
        parameters=[LaunchConfiguration("joy_params")],
        **ctrl_respawn,
    )

    arm_controller_node = Node(
        package="arm_controller",
        executable="arm_controller",
        name="arm_controller",
        output="both",
        parameters=[
            LaunchConfiguration("arm_params"),
            {"urdf_path": PathJoinSubstitution([bringup_share, "urdf", "sekirei.urdf"])},
        ],
        **ctrl_respawn,
    )

    # Perception (Note: qr_detector is moved to camera.launch.py with composition)

    return LaunchDescription([
        crawler_params,
        flipper_params,
        arm_params,
        sensor_gw_params,
        gripper_params,
        arm_driver_params,
        joy_params,
        # Hardware drivers (with respawn)
        crawler_driver_node,
        flipper_driver_node,
        sensor_gateway_node,
        gripper_driver_node,
        arm_driver_node,
        # Control
        joy_controller_node,
        arm_controller_node,
    ])
