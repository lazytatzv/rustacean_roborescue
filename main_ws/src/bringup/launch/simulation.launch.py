"""本番準拠シミュレーション Launch ファイル.
Gazebo Harmonic + SLAM + Nav2 を一括起動し、実機に近い構成を構築する。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory("bringup")

    # ── 引数設定 ──
    use_slam = LaunchConfiguration("use_slam", default="true")
    use_nav2 = LaunchConfiguration("use_nav2", default="true")
    world = LaunchConfiguration(
        "world", default=os.path.join(bringup_dir, "worlds", "rescue_field.sdf")
    )

    # URDF
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(bringup_dir, "urdf", "robot.urdf.xacro")]),
        value_type=str,
    )

    # ── 環境変数 (シミュレーション用) ──
    use_sim_time_env = SetEnvironmentVariable("USE_SIM_TIME", "true")
    # GUI安定化
    qt_platform = SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb")

    # ── Gazebo 起動 ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": [world, " -r --render-engine ogre"]}.items(),
    )

    # ── ロボット状態・ブリッジ ──
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "rescue_robot", "-z", "0.2"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/velodyne_points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    # ── センサ処理 (Lidar) ──
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/velodyne_points"), ("scan", "/scan")],
        parameters=[
            os.path.join(bringup_dir, "config", "pointcloud_to_laserscan.yaml"),
            {"use_sim_time": True},
        ],
    )

    # ── TFブリッジ (Odom発行) ──
    odom_tf_bridge = Node(
        package="bringup",
        executable="odom_tf_bridge.py",
        parameters=[{"use_sim_time": True}],
    )

    # ── SLAM (slam_toolbox) ──
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            os.path.join(bringup_dir, "config", "slam_toolbox_async.yaml"),
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_slam),
    )

    # ── Nav2 (起動を遅らせて安定化) ──
    nav2 = TimerAction(
        period=10.0,
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

    # ── RViz ──
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(bringup_dir, "rviz", "simulation.rviz")],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            use_sim_time_env,
            qt_platform,
            gz_sim,
            robot_state_publisher,
            spawn_entity,
            bridge,
            pointcloud_to_laserscan,
            odom_tf_bridge,
            slam_toolbox,
            nav2,
            rviz,
        ]
    )
