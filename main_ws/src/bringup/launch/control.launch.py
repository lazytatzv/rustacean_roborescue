from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")
    qr_share = FindPackageShare("qr_detector")

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

    # ── Nodes ──
    crawler_driver_node = Node(
        package="crawler_driver",
        executable="crawler_driver_node",
        name="crawler_driver",
        output="screen",
        parameters=[LaunchConfiguration("crawler_params")],
    )

    flipper_driver_node = Node(
        package="flipper_driver",
        executable="flipper_driver_node",
        name="flipper_driver",
        output="screen",
        parameters=[LaunchConfiguration("flipper_params")],
    )

    joy_controller_node = Node(
        package="joy_controller",
        executable="joy_controller_node",
        name="joy_controller",
        output="screen",
    )

    arm_controller_node = Node(
        package="arm_controller",
        executable="arm_controller",
        name="arm_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("arm_params"),
            {"urdf_path": PathJoinSubstitution([bringup_share, "urdf", "sekirei.urdf"])},
        ],
    )

    sensor_gateway_node = Node(
        package="sensor_gateway",
        executable="sensor_gateway",
        name="sensor_gateway",
        output="screen",
        parameters=[LaunchConfiguration("sensor_gw_params")],
    )

    gripper_driver_node = Node(
        package="gripper_driver",
        executable="gripper_driver",
        name="gripper_driver",
        output="screen",
        parameters=[LaunchConfiguration("gripper_params")],
    )

    qr_detector_node = Node(
        package="qr_detector",
        executable="qr_detector_node",
        name="qr_detector",
        output="screen",
        parameters=[{
            "model_dir": PathJoinSubstitution([qr_share, "models"]),
            "publish_compressed": True,
            "jpeg_quality": 60,
            "detection_interval": 1,
        }],
    )

    return LaunchDescription([
        crawler_params,
        flipper_params,
        arm_params,
        sensor_gw_params,
        gripper_params,
        # Hardware drivers
        crawler_driver_node,
        flipper_driver_node,
        sensor_gateway_node,
        gripper_driver_node,
        # Control
        joy_controller_node,
        arm_controller_node,
        # Vision
        qr_detector_node,
    ])
