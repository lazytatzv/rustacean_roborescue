"""シミュレーション Launch ファイル.

Gazebo Harmonic + SLAM + Nav2 を一括起動する。

引数:
  headless   : true  → GPUなし/CI向け (センサーなし・GUIなし)
               false → GPU搭載機向け (LiDAR/IMU有効、nixGL環境は `nixGL gz` で起動)
  use_slam   : SLAM Toolbox を起動するか (default: true)
  use_nav2   : Nav2 を起動するか (default: false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")

    # ── 引数 ──────────────────────────────────────────────────────────────────
    arg_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="true=GPUなし/CI向け(センサーなし・GUIなし) / false=GPU搭載機向け",
    )
    arg_use_slam = DeclareLaunchArgument(
        "use_slam", default_value="true", description="SLAM Toolbox を起動するか"
    )
    arg_use_nav2 = DeclareLaunchArgument(
        "use_nav2", default_value="false", description="Nav2 を起動するか"
    )
    arg_use_gz_gui = DeclareLaunchArgument(
        "use_gz_gui",
        default_value="false",
        description="Gazebo GUI を起動するか (Nix/Wayland では false 推奨)",
    )
    arg_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="RViz を起動するか",
    )
    arg_spawn_delay = DeclareLaunchArgument(
        "spawn_delay_sec",
        default_value="15.0",
        description="Gazebo 起動後にロボットを spawn するまでの待機秒",
    )
    arg_nav2_delay = DeclareLaunchArgument(
        "nav2_delay_sec",
        default_value="30.0",
        description="Nav2 起動を遅延させる秒数",
    )

    headless = LaunchConfiguration("headless")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_gz_gui = LaunchConfiguration("use_gz_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    spawn_delay_sec = LaunchConfiguration("spawn_delay_sec")
    nav2_delay_sec = LaunchConfiguration("nav2_delay_sec")

    # GPU有り: センサー付きワールド, GUI付き
    world_gpu = os.path.join(bringup_dir, "worlds", "rescue_field.sdf")
    # GPU無し: センサーなしワールド, サーバーのみ
    world_headless = os.path.join(bringup_dir, "worlds", "rescue_field_headless.sdf")

    # ── URDF ──────────────────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(bringup_dir, "urdf", "robot.urdf.xacro")]),
        value_type=str,
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────
    # GPU有り: センサー有効ワールド
    # GUI有無は use_gz_gui で切り替える
    gz_partition = SetEnvironmentVariable("GZ_PARTITION", "roborescue")
    ign_partition = SetEnvironmentVariable("IGN_PARTITION", "roborescue")

    # GUIあり
    gz_sim_gpu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"{world_gpu} -r"}.items(),
        condition=IfCondition(
            PythonExpression(["'", headless, "' != 'true' and '", use_gz_gui, "' == 'true'"])
        ),
    )
    # GUIなし (サーバーのみ)
    gz_sim_gpu_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"{world_gpu} -r -s"}.items(),
        condition=IfCondition(
            PythonExpression(["'", headless, "' != 'true' and '", use_gz_gui, "' != 'true'"])
        ),
    )
    # GPU無し: サーバーのみ (-s), GUI無効
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"{world_headless} -r -s"}.items(),
        condition=IfCondition(headless),
    )

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # ── spawn (Gazebo 起動後 5 秒待つ) ────────────────────────────────────────
    spawn_entity = TimerAction(
        period=spawn_delay_sec,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-world",
                    "rescue_field",
                    "-topic",
                    "/robot_description",
                    "-name",
                    "rescue_robot",
                    "-z",
                    "0.1",
                ],
                output="screen",
                additional_env={
                    "GZ_PARTITION": "roborescue",
                    "IGN_PARTITION": "roborescue",
                },
            )
        ],
    )

    # ── Gazebo ↔ ROS 2 ブリッジ ───────────────────────────────────────────────
    # LiDAR は Gazebo の点群を ROS 2 に橋渡しし、ROS 側で /scan に変換する。
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/velodyne_points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                    "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                ],
                output="screen",
            )
        ],
    )

    scan_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                parameters=[os.path.join(bringup_dir, "config", "pointcloud_to_laserscan.yaml")],
                remappings=[
                    ("cloud_in", "/velodyne_points"),
                    ("scan", "/scan"),
                ],
                output="screen",
            )
        ],
    )

    # ── TF ブリッジ: /odom → odom→base_footprint TF ──────────────────────────
    odom_tf_bridge = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="bringup",
                executable="odom_tf_bridge.py",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    # ── SLAM Toolbox ──────────────────────────────────────────────────────────
    slam_toolbox = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                parameters=[
                    os.path.join(bringup_dir, "config", "slam_toolbox_async.yaml"),
                    {
                        "qos_overrides./scan.subscription.reliability": "best_effort",
                        "qos_overrides./scan.subscription.durability": "volatile",
                    },
                    {"use_sim_time": True},
                ],
                condition=IfCondition(
                    PythonExpression(["'", use_slam, "' == 'true' and '", headless, "' != 'true'"])
                ),
            )
        ],
    )

    # ── SLAM Lifecycle Manager (slam_toolbox を自動 activate) ───────────────
    slam_lifecycle_manager = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_slam",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "autostart": True,
                        "bond_timeout": 0.0,
                        "node_names": ["slam_toolbox"],
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_slam,
                            "' == 'true' and '",
                            headless,
                            "' != 'true'",
                        ]
                    )
                ),
            )
        ],
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    nav2 = TimerAction(
        period=nav2_delay_sec,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "nav2.launch.py")
                ),
                launch_arguments={"use_sim_time": "true", "autostart": "true"}.items(),
            )
        ],
        condition=IfCondition(use_nav2),
    )

    # ── RViz (GPU有りのみ) ────────────────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(bringup_dir, "rviz", "simulation.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
            PythonExpression(["'", headless, "' != 'true' and '", use_rviz, "' == 'true'"])
        ),
    )

    return LaunchDescription(
        [
            arg_headless,
            arg_use_slam,
            arg_use_nav2,
            arg_use_gz_gui,
            arg_use_rviz,
            arg_spawn_delay,
            arg_nav2_delay,
            gz_partition,
            ign_partition,
            robot_state_publisher,
            gz_sim_gpu,
            gz_sim_gpu_server,
            gz_sim_headless,
            spawn_entity,
            bridge,
            scan_node,
            odom_tf_bridge,
            slam_toolbox,
            slam_lifecycle_manager,
            nav2,
            rviz,
        ]
    )

