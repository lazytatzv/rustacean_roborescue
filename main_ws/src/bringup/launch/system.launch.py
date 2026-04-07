# ここから他のlaunchファイルを呼び出し、
# このスクリプトでロボットのシステムを起動する

# operator側でfoxglove_bridgeとjoy_nodeを立ち上げる

# systemdに便り過ぎるのもまた複雑性が増し、
# Justfileで頑張るのもまた大変なので、なるべくlaunchから
# zenohdもlaunchから起動する

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def _load_launch_config(bringup_dir: str) -> dict:
    config_path = os.path.join(bringup_dir, "config", "launch_config.yaml")
    if not os.path.isfile(config_path):
        return {}

    try:
        with open(config_path) as f:
            loaded = yaml.safe_load(f)
    except Exception:
        return {}

    return loaded if isinstance(loaded, dict) else {}


def _safe_include(
    launch_path: str,
    *,
    launch_arguments=None,
    condition=None,
    label: str = "",
):
    if not os.path.isfile(launch_path):
        target = label if label else launch_path
        return LogInfo(msg=f"[WARN][system.launch] missing launch file, skip: {target}")

    kwargs = {}
    if launch_arguments is not None:
        kwargs["launch_arguments"] = launch_arguments.items()
    if condition is not None:
        kwargs["condition"] = condition

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        **kwargs,
    )


def _b(val: bool) -> str:
    return "true" if val else "false"


def generate_launch_description():
    try:
        bringup_dir = get_package_share_directory("bringup")
        cfg = _load_launch_config(bringup_dir)

        # 各サブLaunchファイルのパス
        network_launch = os.path.join(bringup_dir, "launch", "network.launch.py")
        perception_launch = os.path.join(bringup_dir, "launch", "perception.launch.py")
        control_launch = os.path.join(bringup_dir, "launch", "control.launch.py")
        camera_launch = os.path.join(bringup_dir, "launch", "camera.launch.py")
        nav2_launch = os.path.join(bringup_dir, "launch", "nav2.launch.py")
        audio_launch = os.path.join(bringup_dir, "launch", "audio.launch.py")

        default_use_nav2 = _b(cfg.get("use_nav2", False))
        default_use_audio = _b(cfg.get("use_audio", True))

        # Nav2 + SLAM 整合チェック: use_nav2=true かつ use_lidar=false は起動失敗の原因
        _nav2_on = cfg.get("use_nav2", False)
        _lidar_on = cfg.get("use_lidar", True)
        _nav2_slam_warn = (
            LogInfo(
                msg=(
                    "[ERROR][system.launch] use_nav2=true だが use_lidar=false です。"
                    " Nav2 の global_costmap は map フレームを必要とし、"
                    " map フレームは SLAM (use_lidar=true) が提供します。"
                    " global_costmap が起動失敗します。"
                )
            )
            if _nav2_on and not _lidar_on
            else None
        )
        default_use_lidar = _b(cfg.get("use_lidar", True))
        default_use_velodyne = _b(cfg.get("use_velodyne", cfg.get("use_lidar", True)))
        default_use_camera = _b(cfg.get("use_camera", True))
        default_use_crawler = _b(cfg.get("use_crawler", True))
        default_use_arm = _b(cfg.get("use_arm", True))
        default_use_flipper = _b(cfg.get("use_flipper", True))
        default_use_imu = _b(cfg.get("use_imu", True))
        default_use_t265_odom = _b(cfg.get("use_t265_odom", False))
        default_use_depth_guard = _b(cfg.get("use_depth_guard", False))

        arg_use_nav2 = DeclareLaunchArgument("use_nav2", default_value=default_use_nav2)
        arg_use_audio = DeclareLaunchArgument("use_audio", default_value=default_use_audio)
        arg_use_lidar = DeclareLaunchArgument("use_lidar", default_value=default_use_lidar)
        arg_use_velodyne = DeclareLaunchArgument("use_velodyne", default_value=default_use_velodyne)
        arg_use_camera = DeclareLaunchArgument("use_camera", default_value=default_use_camera)
        arg_use_crawler = DeclareLaunchArgument("use_crawler", default_value=default_use_crawler)
        arg_use_arm = DeclareLaunchArgument("use_arm", default_value=default_use_arm)
        arg_use_flipper = DeclareLaunchArgument("use_flipper", default_value=default_use_flipper)
        arg_use_imu = DeclareLaunchArgument("use_imu", default_value=default_use_imu)
        arg_use_t265_odom = DeclareLaunchArgument(
            "use_t265_odom", default_value=default_use_t265_odom
        )
        arg_use_depth_guard = DeclareLaunchArgument(
            "use_depth_guard", default_value=default_use_depth_guard
        )

        use_nav2 = LaunchConfiguration("use_nav2")
        use_audio = LaunchConfiguration("use_audio")
        use_lidar = LaunchConfiguration("use_lidar")
        use_velodyne = LaunchConfiguration("use_velodyne")
        use_camera = LaunchConfiguration("use_camera")
        use_crawler = LaunchConfiguration("use_crawler")
        use_arm = LaunchConfiguration("use_arm")
        use_flipper = LaunchConfiguration("use_flipper")
        use_imu = LaunchConfiguration("use_imu")
        use_t265_odom = LaunchConfiguration("use_t265_odom")
        use_depth_guard = LaunchConfiguration("use_depth_guard")
        # ── Robot State Publisher (URDF → TF: base_link→各センサ/アーム/フリッパ) ──
        urdf_file = os.path.join(bringup_dir, "urdf", "robot.urdf.xacro")
        robot_state_publisher_actions = []
        if os.path.isfile(urdf_file):
            robot_description = ParameterValue(
                Command(["xacro ", urdf_file]),
                value_type=str,
            )
            robot_state_publisher_actions.append(
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="robot_state_publisher",
                    output="screen",
                    respawn=True,
                    respawn_delay=3.0,
                    parameters=[
                        {
                            "robot_description": robot_description,
                            "use_sim_time": False,
                        }
                    ],
                )
            )
            # crawler/arm/flipper が全て無効のとき /joint_states を誰も publish しないため
            # robot_state_publisher の TF ツリーが壊れる。ゼロ埋めでフォールバックする。
            robot_state_publisher_actions.append(
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher_fallback",
                    output="screen",
                    parameters=[{"robot_description": robot_description}],
                    condition=IfCondition(
                        PythonExpression(
                            [
                                "'",
                                use_crawler,
                                "' == 'false' and '",
                                use_arm,
                                "' == 'false' and '",
                                use_flipper,
                                "' == 'false'",
                            ]
                        )
                    ),
                )
            )
        else:
            robot_state_publisher_actions.append(
                LogInfo(
                    msg=(
                        "[WARN][system.launch] missing URDF xacro, skip robot_state_publisher: "
                        f"{urdf_file}"
                    )
                )
            )

        # foxglove bridgeはOperator側で立ち上げる想定
        # # Foxglove Bridge — Foxglove Studio が WebSocket (ws://robot_ip:8765) で接続
        # foxglove_bridge_node = Node(
        #     package='foxglove_bridge',
        #     executable='foxglove_bridge',
        #     name='foxglove_bridge',
        #     output='screen',
        #     parameters=[{
        #         'port': 8765,
        #         'address': '0.0.0.0',
        #         'send_buffer_limit': 10000000,       # 10 MB
        #         'max_qos_depth': 10,
        #         'num_threads': 0,                    # 0 = auto
        #         'use_compression': False,
        #     }],
        # )

        # Zenoh設定ファイルの絶対パスを確定
        zenoh_robot_config = os.path.realpath(
            os.path.join(bringup_dir, "config", "zenoh_robot.json5")
        )

        actions = [
            LogInfo(msg="[system.launch] starting with fail-safe guards"),
            *([_nav2_slam_warn] if _nav2_slam_warn else []),
            arg_use_nav2,
            arg_use_audio,
            arg_use_lidar,
            arg_use_velodyne,
            arg_use_camera,
            arg_use_crawler,
            arg_use_arm,
            arg_use_flipper,
            arg_use_imu,
            arg_use_t265_odom,
            arg_use_depth_guard,
            # ==========================================
            # Zenoh / RMW 設定 (すべてのノードより前に実行)
            # ==========================================
            SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
            SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_zenoh_cpp"),
            SetEnvironmentVariable("ZENOH_ROUTER_CHECK_ATTEMPTS", "-1"),
            SetEnvironmentVariable("ZENOH_SESSION_CONFIG_URI", zenoh_robot_config),
            # 0. Robot State Publisher (URDF TF: base_link→各センサ/アーム/フリッパ)
            *robot_state_publisher_actions,
            # 1. Foxglove Bridge (WebSocket :8765)
            # foxglove_bridge_node,
            # ネットワーク（Zenoh）の起動
            _safe_include(
                network_launch,
                label="network.launch.py",
            ),
            # オーディオ (signaling + robot webrtc)
            _safe_include(
                audio_launch,
                label="audio.launch.py",
                launch_arguments={"use_audio": use_audio},
            ),
            # 認識・SLAM系の起動
            _safe_include(
                perception_launch,
                label="perception.launch.py",
                launch_arguments={
                    "use_slam": use_lidar,
                    "use_lidar": use_lidar,
                    "use_velodyne": use_velodyne,
                    "use_scan": use_lidar,
                    "use_rviz": "false",
                    "use_t265_odom": use_t265_odom,
                },
            ),
            # カメラの起動 (qr_detector が /camera/image_raw を必要とする)
            _safe_include(
                camera_launch,
                label="camera.launch.py",
                launch_arguments={
                    "use_camera": use_camera,
                    "use_depth_guard": use_depth_guard,
                },
            ),
            # 制御系の起動
            _safe_include(
                control_launch,
                label="control.launch.py",
                launch_arguments={
                    "use_crawler": use_crawler,
                    "use_arm": use_arm,
                    "use_flipper": use_flipper,
                    "use_imu": use_imu,
                },
            ),
            # Nav2 自律走行（オプション）
            _safe_include(
                nav2_launch,
                label="nav2.launch.py",
                condition=IfCondition(use_nav2),
            ),
        ]

        return LaunchDescription(actions)
    except Exception as e:
        # generate_launch_description で例外を投げると launch 全体が即終了するため、
        # ここで握りつぶして明示ログだけ残す。
        return LaunchDescription(
            [
                LogInfo(msg=f"[WARN][system.launch] recovered from fatal setup error: {e}"),
                LogInfo(msg="[system.launch] no actions started due to setup error"),
            ]
        )
