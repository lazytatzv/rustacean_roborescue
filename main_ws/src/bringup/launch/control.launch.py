import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _generate_urdf(use_real_hw: bool, dxl_model_folder: str) -> tuple[str, str]:
    """sekirei_moveit.xacro を展開して (urdf_string, urdf_file_path) を返す。
    Rust arm_controller はファイルパスで読むため /tmp に書き出す。
    ros2_control と Rust IK で同じ xacro を共有する。
    """
    import subprocess
    import tempfile

    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd
        xacro_path = os.path.join(
            _gpsd("sekirei_moveit"), "urdf", "sekirei_moveit.xacro"
        )
        if os.path.isfile(xacro_path):
            args = [
                "xacro", xacro_path,
                f"use_real_hw:={'true' if use_real_hw else 'false'}",
            ]
            if dxl_model_folder:
                args.append(f"dynamixel_model_folder:={dxl_model_folder}")
            result = subprocess.run(args, capture_output=True, text=True, check=True)
            urdf_str = result.stdout
            # arm_controller (Rust) 用に一時ファイルへ書き出す
            tmp = tempfile.NamedTemporaryFile(
                mode="w", suffix=".urdf", prefix="sekirei_moveit_",
                delete=False
            )
            tmp.write(urdf_str)
            tmp.flush()
            return urdf_str, tmp.name
    except Exception as e:
        print(f"[control.launch] xacro failed ({e}), falling back to sekirei.urdf")

    # フォールバック: sekirei.urdf
    urdf_path = os.path.join(get_package_share_directory("bringup"), "urdf", "sekirei.urdf")
    with open(urdf_path) as f:
        urdf_str = f.read().replace("DYNAMIXEL_MODEL_FOLDER_PLACEHOLDER", dxl_model_folder)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".urdf", prefix="sekirei_fallback_", delete=False
    )
    tmp.write(urdf_str)
    tmp.flush()
    return urdf_str, tmp.name


def _load_device_path(config_path: str, root_key: str, param_key: str) -> str:
    if not os.path.isfile(config_path):
        return ""
    try:
        with open(config_path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = data.get(root_key, {}).get("ros__parameters", {})
        value = params.get(param_key, "")
        return value if isinstance(value, str) else ""
    except Exception:
        return ""


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("bringup")
    bringup_share = FindPackageShare("bringup")

    crawler_cfg = os.path.join(bringup_dir, "config", "crawler_driver.yaml")
    flipper_cfg = os.path.join(bringup_dir, "config", "flipper_driver.yaml")
    sensor_cfg = os.path.join(bringup_dir, "config", "sensor_gateway.yaml")
    arm_cfg = os.path.join(bringup_dir, "config", "arm_gripper_driver.yaml")

    crawler_port = _load_device_path(crawler_cfg, "crawler_driver", "serial_port")
    flipper_port = _load_device_path(flipper_cfg, "flipper_driver", "port_name")
    sensor_port = _load_device_path(sensor_cfg, "sensor_gateway", "serial_port")
    arm_port = _load_device_path(arm_cfg, "arm_gripper_driver", "port_name")

    crawler_available = bool(crawler_port) and os.path.exists(crawler_port)
    flipper_available = bool(flipper_port) and os.path.exists(flipper_port)
    sensor_available = bool(sensor_port) and os.path.exists(sensor_port)
    arm_available = bool(arm_port) and os.path.exists(arm_port)

    availability_warnings = []
    if crawler_port and not crawler_available:
        availability_warnings.append(
            LogInfo(
                msg=f"[WARN][control.launch] crawler device not found: {crawler_port} -> disabling crawler"
            )
        )
    if flipper_port and not flipper_available:
        availability_warnings.append(
            LogInfo(
                msg=f"[WARN][control.launch] flipper device not found: {flipper_port} -> disabling flipper"
            )
        )
    if sensor_port and not sensor_available:
        availability_warnings.append(
            LogInfo(
                msg=f"[WARN][control.launch] imu gateway device not found: {sensor_port} -> disabling imu gateway"
            )
        )
    if arm_port and not arm_available:
        availability_warnings.append(
            LogInfo(
                msg=f"[WARN][control.launch] arm device not found: {arm_port} -> disabling arm drivers"
            )
        )

    # ── Launch arguments ──────────────────────────────────────────────────
    arg_crawler_params = DeclareLaunchArgument(
        "crawler_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "crawler_driver.yaml"]),
    )
    arg_flipper_params = DeclareLaunchArgument(
        "flipper_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "flipper_driver.yaml"]),
    )
    arg_joy_params = DeclareLaunchArgument(
        "joy_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "joy_controller.yaml"]),
    )

    arg_sensor_gw_params = DeclareLaunchArgument(
        "sensor_gw_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "sensor_gateway.yaml"]),
    )
    arg_arm_gripper_driver_params = DeclareLaunchArgument(
        "arm_gripper_driver_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "arm_gripper_driver.yaml"]),
    )
    arg_arm_params = DeclareLaunchArgument(
        "arm_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "arm_controller.yaml"]),
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
            "MoveIt2 を使う場合は arm_backend:=ros2_control use_moveit:=true で起動し、\n"
            "手動切り替えが必要な場合:\n"
            "  ros2 control switch_controllers \\\n"
            "    --deactivate forward_position_controller \\\n"
            "    --activate   joint_trajectory_controller"
        ),
    )
    arg_use_moveit = DeclareLaunchArgument(
        "use_moveit",
        default_value="false",
        description=(
            "MoveIt Servo (move_group + servo_node) を起動する。\n"
            "arm_backend:=ros2_control と組み合わせて使用。\n"
            "joy_controller の SERVO モード (SHARE x3) と連携。"
        ),
    )

    ros2_control_available = True
    ros2_control_warning = None
    robot_description = ""
    urdf_file_path = ""
    try:
        dxl_model_folder = os.path.join(
            get_package_share_directory("dynamixel_hardware_interface"),
            "param",
            "dxl_model",
        )
        robot_description, urdf_file_path = _generate_urdf(
            use_real_hw=True, dxl_model_folder=dxl_model_folder
        )
    except Exception:
        ros2_control_available = False
        ros2_control_warning = LogInfo(
            msg=(
                "[WARN][control.launch] dynamixel_hardware_interface unavailable; "
                "falling back to arm_backend=direct"
            )
        )

    # direct モードでも同じ xacro (use_real_hw=false) を arm_controller に渡す
    if not urdf_file_path:
        _, urdf_file_path = _generate_urdf(use_real_hw=False, dxl_model_folder="")

    use_ros2_control = PythonExpression(
        [
            "'",
            LaunchConfiguration("arm_backend"),
            "' == 'ros2_control' and '",
            LaunchConfiguration("use_arm"),
            "' == 'true' and ",
            str(ros2_control_available),
            " and ",
            str(arm_available),
        ]
    )

    use_crawler_effective = PythonExpression(
        ["'", LaunchConfiguration("use_crawler"), "' == 'true' and ", str(crawler_available)]
    )
    use_flipper_effective = PythonExpression(
        ["'", LaunchConfiguration("use_flipper"), "' == 'true' and ", str(flipper_available)]
    )
    use_imu_effective = PythonExpression(
        ["'", LaunchConfiguration("use_imu"), "' == 'true' and ", str(sensor_available)]
    )
    use_arm_effective = PythonExpression(
        ["'", LaunchConfiguration("use_arm"), "' == 'true' and ", str(arm_available)]
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
        condition=IfCondition(use_crawler_effective),
        **hw_respawn,
    )
    flipper_driver_node = Node(
        package="flipper_driver",
        executable="flipper_driver_node",
        name="flipper_driver",
        output="both",
        parameters=[LaunchConfiguration("flipper_params")],
        condition=IfCondition(use_flipper_effective),
        **flipper_respawn,
    )
    sensor_gateway_node = Node(
        package="sensor_gateway",
        executable="sensor_gateway",
        name="sensor_gateway",
        output="both",
        parameters=[LaunchConfiguration("sensor_gw_params")],
        condition=IfCondition(use_imu_effective),
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
            {"robot_description": robot_description},
        ],
        condition=IfCondition(use_arm_effective),
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
                    "' == 'true' and ",
                    str(arm_available),
                    " and ('",
                    LaunchConfiguration("arm_backend"),
                    "' != 'ros2_control' or not ",
                    str(ros2_control_available),
                    ")",
                ]
            )
        ),
        **hw_respawn,
    )

    # ── ros2_control モード ───────────────────────────────────────────────
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
                    "' == 'true' and ",
                    str(arm_available),
                    " and '",
                    LaunchConfiguration("arm_backend"),
                    "' == 'ros2_control'",
                ]
            )
        ),
    )
    # robot_state_publisher はここでは起動しない。
    # system.launch.py が robot.urdf.xacro で RSP を起動済み。
    # controller_manager は上記 parameters=["robot_description": robot_description] で
    # sekirei.urdf (ros2_control タグ付き) を直接パラメータとして受け取るため RSP 不要。

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

    # MoveIt Servo (move_group + servo_node) — only when use_moveit=true AND ros2_control
    moveit_servo_available = True
    moveit_servo_warning = None
    try:
        get_package_share_directory("sekirei_moveit")
    except Exception:
        moveit_servo_available = False
        moveit_servo_warning = LogInfo(
            msg="[WARN][control.launch] sekirei_moveit not found; use_moveit ignored"
        )

    use_moveit_effective = PythonExpression(
        [
            "'",
            LaunchConfiguration("use_moveit"),
            "' == 'true' and '",
            LaunchConfiguration("arm_backend"),
            "' == 'ros2_control' and ",
            str(moveit_servo_available),
        ]
    )

    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sekirei_moveit")
                if moveit_servo_available
                else "/dev/null",
                "launch",
                "moveit_servo.launch.py",
            )
        ),
        condition=IfCondition(use_moveit_effective),
    ) if moveit_servo_available else LogInfo(msg="")

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
            arg_use_moveit,
            # 依存不足時のフォールバック告知
            *([ros2_control_warning] if ros2_control_warning is not None else []),
            *([moveit_servo_warning] if moveit_servo_warning is not None else []),
            *availability_warnings,
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
            spawn_joint_state_broadcaster,
            spawn_forward_position_controller,
            spawn_joint_trajectory_controller,
            arm_cmd_bridge_node,
            # MoveIt Servo (use_moveit:=true かつ arm_backend:=ros2_control の場合)
            moveit_servo_launch,
        ]
    )
