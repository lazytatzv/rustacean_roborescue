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
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
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

    headless = LaunchConfiguration("headless")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")

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
    # GPU有り: GUIあり・センサー有効
    gz_sim_gpu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world_gpu, " -r"]}.items(),
        condition=UnlessCondition(headless),
    )
    # GPU無し: サーバーのみ (-s), GUI無効
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world_headless, " -r -s"]}.items(),
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
        period=5.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "rescue_robot",
                    "-z",
                    "0.1",
                ],
                output="screen",
            )
        ],
    )

    # ── Gazebo ↔ ROS 2 ブリッジ ───────────────────────────────────────────────
    # joint_statesはGazeboのJointStatePublisherプラグインがROS2に直接publish
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
                    "/velodyne_points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
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

    # ── PointCloud → LaserScan ────────────────────────────────────────────────
    pointcloud_to_laserscan = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                remappings=[("cloud_in", "/velodyne_points"), ("scan", "/scan")],
                parameters=[
                    os.path.join(bringup_dir, "config", "pointcloud_to_laserscan.yaml"),
                    {"use_sim_time": True},
                ],
                condition=UnlessCondition(headless),
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
                    {"use_sim_time": True},
                ],
                condition=IfCondition(
                    PythonExpression(["'", use_slam, "' == 'true' and '", headless, "' != 'true'"])
                ),
            )
        ],
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────────
    nav2 = TimerAction(
        period=15.0,
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
        condition=UnlessCondition(headless),
    )

    return LaunchDescription(
        [
            arg_headless,
            arg_use_slam,
            arg_use_nav2,
            robot_state_publisher,
            gz_sim_gpu,
            gz_sim_headless,
            spawn_entity,
            bridge,
            odom_tf_bridge,
            pointcloud_to_laserscan,
            slam_toolbox,
            nav2,
            rviz,
        ]
    )
