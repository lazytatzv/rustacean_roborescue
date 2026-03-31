import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_robot_description() -> str:
    """URDF を読み込み DYNAMIXEL_MODEL_FOLDER_PLACEHOLDER を実パスに置換して返す。"""
    urdf_path = os.path.join(get_package_share_directory("bringup"), "urdf", "sekirei.urdf")
    dxl_model_folder = os.path.join(
        get_package_share_directory("dynamixel_hardware_interface"),
        "param",
        "dxl_model",
    )
    with open(urdf_path) as f:
        return f.read().replace("DYNAMIXEL_MODEL_FOLDER_PLACEHOLDER", dxl_model_folder)


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")

    # ── Launch arguments ──────────────────────────────────────────────────
    arg_crawler_params = DeclareLaunchArgument(
        "crawler_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "crawler_driver.yaml"]),
    )
    arg_flipper_params = DeclareLaunchArgument(
        "flipper_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "flipper_driver.yaml"]),
    )
    arg_arm_params = DeclareLaunchArgument(
        "arm_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "arm_controller.yaml"]),
    )
    arg_sensor_gw_params = DeclareLaunchArgument(
        "sensor_gw_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "sensor_gateway.yaml"]),
    )
    arg_arm_gripper_driver_params = DeclareLaunchArgument(
        "arm_gripper_driver_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "arm_gripper_driver.yaml"]),
    )
    arg_joy_params = DeclareLaunchArgument(
        "joy_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "joy_controller.yaml"]),
    )
    arg_use_crawler = DeclareLaunchArgument(
        "use_crawler", default_value="true", description="Roboclaw 走行ドライバを有効にする"
    )
    arg_use_arm = DeclareLaunchArgument(
        "use_arm", default_value="true", description="アームドライバ + IK を有効にする"
    )
    arg_use_flipper = DeclareLaunchArgument(
        "use_flipper", default_value="true", description="フリッパドライバを有効にする"
    )
    arg_use_imu = DeclareLaunchArgument(
        "use_imu", default_value="true", description="STM32 IMU (sensor_gateway) を有効にする"
    )
    arg_arm_backend = DeclareLaunchArgument(
        "arm_backend",
        default_value="direct",
        description=(
            "アームドライバのバックエンド選択。\n"
            "  direct       : Rust arm_driver で直接 Dynamixel を制御 (デフォルト)\n"
            "  ros2_control : dynamixel_hardware_interface + controller_manager を使用。\n"
            "                 MoveIt2 / joint_trajectory_controller が利用可能になる。\n"
            "MoveIt2 を使う場合は起動後に:\n"
            "  ros2 control switch_controllers \\\n"
            "    --deactivate forward_position_controller \\\n"
            "    --activate   joint_trajectory_controller"
        ),
    )

    use_ros2_control = PythonExpression(
        ["'", LaunchConfiguration("arm_backend"), "' == 'ros2_control'"]
    )

    hw_respawn = {"respawn": True, "respawn_delay": 3.0}
    # フリッパーは Wheel Mode 中に crash すると motors が spinning し続けるため
    # respawn_delay を短くして停止までの時間を最小化する
    flipper_respawn = {"respawn": True, "respawn_delay": 1.0}
    ctrl_respawn = {"respawn": True, "respawn_delay": 2.0}

    # ── 共通: ハードウェアドライバ ────────────────────────────────────────
    crawler_driver_node = Node(
        package="crawler_driver",
        executable="crawler_driver_node",
        name="crawler_driver",
        output="both",
        parameters=[LaunchConfiguration("crawler_params")],
        condition=IfCondition(LaunchConfiguration("use_crawler")),
        **hw_respawn,
    )
    flipper_driver_node = Node(
        package="flipper_driver",
        executable="flipper_driver_node",
        name="flipper_driver",
        output="both",
        parameters=[LaunchConfiguration("flipper_params")],
        condition=IfCondition(LaunchConfiguration("use_flipper")),
        **flipper_respawn,
    )
    sensor_gateway_node = Node(
        package="sensor_gateway",
        executable="sensor_gateway",
        name="sensor_gateway",
        output="both",
        parameters=[LaunchConfiguration("sensor_gw_params")],
        condition=IfCondition(LaunchConfiguration("use_imu")),
        **hw_respawn,
    )

    # ── 共通: コントローラ ────────────────────────────────────────────────
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
        condition=IfCondition(LaunchConfiguration("use_arm")),
        **ctrl_respawn,
    )

    # ── direct モード: Rust arm_driver ────────────────────────────────────
    arm_driver_direct = Node(
        package="arm_driver",
        executable="arm_driver",
        name="arm_gripper_driver",
        output="both",
        parameters=[LaunchConfiguration("arm_gripper_driver_params")],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_arm"),
                    "' == 'true'",
                    " and '",
                    LaunchConfiguration("arm_backend"),
                    "' != 'ros2_control'",
                ]
            )
        ),
        **hw_respawn,
    )

    # ── ros2_control モード ───────────────────────────────────────────────
    robot_description = _load_robot_description()
    controllers_yaml = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "ros2_control_controllers.yaml",
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_yaml,
        ],
        output="both",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_arm"),
                    "' == 'true'",
                    " and '",
                    LaunchConfiguration("arm_backend"),
                    "' == 'ros2_control'",
                ]
            )
        ),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    # コントローラのスポーン (spawner は controller_manager の起動を自動で待つ)
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )
    # テレオペ用: デフォルトでアクティブ化
    spawn_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )
    # MoveIt2 用: ロードのみ (inactive) — 使う時に switch_controllers で切り替え
    spawn_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    # bridge: /arm_joint_commands (JointState) → /forward_position_controller/commands
    arm_cmd_bridge_node = Node(
        package="bringup",
        executable="arm_cmd_bridge.py",
        name="arm_cmd_bridge",
        output="screen",
        condition=IfCondition(use_ros2_control),
        **ctrl_respawn,
    )

    return LaunchDescription(
        [
            # 引数
            arg_use_crawler,
            arg_use_arm,
            arg_use_flipper,
            arg_use_imu,
            arg_crawler_params,
            arg_flipper_params,
            arg_arm_params,
            arg_sensor_gw_params,
            arg_arm_gripper_driver_params,
            arg_joy_params,
            arg_arm_backend,
            # 共通
            crawler_driver_node,
            flipper_driver_node,
            sensor_gateway_node,
            joy_controller_node,
            arm_controller_node,
            # direct モード
            arm_driver_direct,
            # ros2_control モード
            controller_manager_node,
            robot_state_publisher_node,
            spawn_joint_state_broadcaster,
            spawn_forward_position_controller,
            spawn_joint_trajectory_controller,
            arm_cmd_bridge_node,
        ]
    )
