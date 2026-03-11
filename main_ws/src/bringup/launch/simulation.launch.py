"""Gazebo Harmonic シミュレーション Launch ファイル.

gz sim (Gazebo Harmonic) + ros_gz_bridge + robot_state_publisher を起動する。
実機のハードウェアドライバの代わりに Gazebo プラグインがセンサ/odom を提供し、
ros_gz_bridge で gz トピックを ROS 2 トピックへ変換する。

使い方:
  # シミュレーション単体
  ros2 launch bringup simulation.launch.py

  # SLAM + Nav2 付き
  ros2 launch bringup simulation.launch.py use_slam:=true use_nav2:=true

  # ヘッドレス (GUI なし)
  ros2 launch bringup simulation.launch.py headless:=true
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
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")
    bringup_dir = get_package_share_directory("bringup")

    # ── Arguments ──
    world_file = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([bringup_share, "worlds", "rescue_field.sdf"]),
        description="Gazebo Harmonic ワールドファイルのパス (.sdf)",
    )
    use_slam_arg = DeclareLaunchArgument("use_slam", default_value="true")
    use_nav2_arg = DeclareLaunchArgument("use_nav2", default_value="false")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    headless_arg = DeclareLaunchArgument("headless", default_value="false",
                                         description="true でヘッドレス (GUI なし) 起動")

    urdf_file = os.path.join(bringup_dir, "urdf", "robot.urdf.xacro")
    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str,
    )

    # ── sim_time を全ノードで有効化 ──
    use_sim_time = SetEnvironmentVariable("USE_SIM_TIME", "true")

    # ── Gazebo がメッシュ等を解決できるようリソースパスを設定 ──
    gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(bringup_dir, os.pardir),
    )

    # ── Gazebo Harmonic (gz sim) ──
    # headless=true  → server only + EGL
    # headless=false → GUI, ogre レンダラ (Nix 環境で ogre2/Vulkan が動かないため)
    gz_args = PythonExpression([
        "'-s -r --headless-rendering ' + '",
        LaunchConfiguration("world"),
        "' if '",
        LaunchConfiguration("headless"),
        "' == 'true' else '-r --render-engine ogre ' + '",
        LaunchConfiguration("world"),
        "'",
    ])
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": gz_args,
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Robot State Publisher ──
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    # ── Spawn robot in Gazebo ──
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entity",
        arguments=[
            "-topic", "robot_description",
            "-name", "rescue_robot",
            "-x", "0.0", "-y", "0.0", "-z", "0.2",
        ],
        output="screen",
    )

    # ── ros_gz_bridge: gz topics ↔ ROS 2 topics ──
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            # cmd_vel: ROS 2 → Gazebo
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            # odom: Gazebo → ROS 2
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # LiDAR: Gazebo → ROS 2
            "/velodyne_points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            # IMU: Gazebo → ROS 2
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # Camera: Gazebo → ROS 2
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            # Joint states: Gazebo → ROS 2
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            # Clock: Gazebo → ROS 2
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Arm joint position commands: ROS 2 → Gazebo
            "/arm_joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm_joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm_joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm_joint4/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm_joint5/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/arm_joint6/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
        ],
    )

    # ── CrawlerVelocity → Twist ブリッジ (teleop 用) ──
    crawler_bridge = Node(
        package="bringup",
        executable="crawler_vel_bridge.py",
        name="crawler_vel_bridge",
        output="screen",
        parameters=[{
            "track_width": 0.4,
            "use_sim_time": True,
        }],
    )

    # ── Arm Gazebo Bridge: JointState → per-joint cmd_pos for Gazebo ──
    arm_gz_bridge = Node(
        package="bringup",
        executable="arm_gz_bridge.py",
        name="arm_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── Odom → TF Bridge: /odom → odom→base_footprint TF ──
    odom_tf_bridge = Node(
        package="bringup",
        executable="odom_tf_bridge.py",
        name="odom_tf_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── Arm Controller (IK → target positions) ──
    arm_controller = Node(
        package="arm_controller",
        executable="arm_controller",
        name="arm_controller",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"urdf_path": os.path.join(bringup_dir, "urdf", "sekirei.urdf")},
        ],
    )

    # ── Joy Controller (teleop、シミュレーションでも使える) ──
    joy_controller = Node(
        package="joy_controller",
        executable="joy_controller_node",
        name="joy_controller",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ── PointCloud → LaserScan ──
    scan_params_file = os.path.join(bringup_dir, "config", "pointcloud_to_laserscan.yaml")
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/velodyne_points"),
            ("scan", "/scan"),
        ],
        parameters=[
            scan_params_file,
            {"use_sim_time": True},
        ],
    )

    # ── SLAM Toolbox (オプション) ──
    slam_params_file = os.path.join(bringup_dir, "config", "slam_toolbox_async.yaml")
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_sim_time": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_footprint",
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )

    # ── SLAM Toolbox Lifecycle Manager ──
    # Jazzy では async_slam_toolbox_node がライフサイクルノードなので
    # activate しないと /scan を購読しない
    slam_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["slam_toolbox"],
        }],
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )

    # ── map→odom 静的TF (headless センサなし時のフォールバック) ──
    # SLAM が /scan データを受信すれば /tf で上書きされる
    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_slam")),
    )

    # ── Nav2 (オプション) ──
    # SLAM Toolbox が activate されて map→odom TF を出すまで待つ
    nav2_launch_file = os.path.join(bringup_dir, "launch", "nav2.launch.py")
    nav2 = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration("use_nav2")),
    )

    # ── RViz ──
    rviz_config = os.path.join(bringup_dir, "rviz", "simulation.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        world_file,
        use_slam_arg,
        use_nav2_arg,
        use_rviz_arg,
        headless_arg,
        use_sim_time,
        gz_resource_path,
        # Gazebo Harmonic
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        # gz ↔ ROS 2 Bridge
        bridge,
        odom_tf_bridge,
        # Bridge + Teleop
        crawler_bridge,
        joy_controller,
        # Arm control (IK + Gazebo bridge)
        arm_controller,
        arm_gz_bridge,
        # Perception
        pointcloud_to_laserscan,
        slam_toolbox,
        slam_lifecycle_manager,
        static_map_tf,
        # Navigation
        nav2,
        # Visualization
        rviz,
    ])
