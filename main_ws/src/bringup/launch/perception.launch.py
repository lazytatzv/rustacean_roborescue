import ctypes
import os


def _can_load_libtbb12() -> bool:
    # First try normal dynamic loader lookup.
    try:
        ctypes.CDLL("libtbb.so.12")
        return True
    except OSError:
        pass

    # Then try explicit paths provided by nix/dev shells.
    candidates = []
    tbb_root = os.environ.get("TBBROOT", "")
    if tbb_root:
        candidates.append(os.path.join(tbb_root, "lib", "libtbb.so.12"))

    for var_name in ("LD_LIBRARY_PATH", "NIX_LD_LIBRARY_PATH"):
        for lib_dir in os.environ.get(var_name, "").split(":"):
            if lib_dir:
                candidates.append(os.path.join(lib_dir, "libtbb.so.12"))

    for candidate in candidates:
        if os.path.exists(candidate):
            try:
                ctypes.CDLL(candidate)
                return True
            except OSError:
                continue

    return False


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")

    kiss_icp_available = _can_load_libtbb12()

    config_path = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution([bringup_share, "config", "spark_fast_lio_min.yaml"]),
    )
    lidar_topic_raw = DeclareLaunchArgument("lidar_topic_raw", default_value="/velodyne_points")
    # timestamp_unit=0 (SEC) により fix ノード不要。戻す場合は
    # use_time_fix:=true lidar_topic:=/velodyne_points_fixed
    lidar_topic = DeclareLaunchArgument("lidar_topic", default_value="/velodyne_points")
    imu_topic = DeclareLaunchArgument("imu_topic", default_value="/imu/data")
    map_frame = DeclareLaunchArgument("map_frame", default_value="odom")
    base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    lidar_frame = DeclareLaunchArgument("lidar_frame", default_value="velodyne")
    kiss_icp_params = DeclareLaunchArgument(
        "kiss_icp_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "kiss_icp.yaml"]),
    )
    use_velodyne = DeclareLaunchArgument("use_velodyne", default_value="false")
    velodyne_driver_params = DeclareLaunchArgument(
        "velodyne_driver_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "velodyne_driver.yaml"]),
    )
    velodyne_pointcloud_params = DeclareLaunchArgument(
        "velodyne_pointcloud_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "velodyne_pointcloud.yaml"]),
    )
    velodyne_calibration = DeclareLaunchArgument(
        "velodyne_calibration",
        default_value=PathJoinSubstitution(
            [FindPackageShare("velodyne_pointcloud"), "params", "VLP16db.yaml"]
        ),
    )

    use_scan = DeclareLaunchArgument("use_scan", default_value="true")
    scan_topic = DeclareLaunchArgument("scan_topic", default_value="/scan")
    scan_params = DeclareLaunchArgument(
        "scan_params",
        default_value=PathJoinSubstitution(
            [bringup_share, "config", "pointcloud_to_laserscan.yaml"]
        ),
    )

    use_time_fix = DeclareLaunchArgument("use_time_fix", default_value="false")
    time_fix_params = DeclareLaunchArgument(
        "time_fix_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "fix_pointcloud_time.yaml"]),
    )

    use_dummy_imu = DeclareLaunchArgument("use_dummy_imu", default_value="false")
    dummy_imu_params = DeclareLaunchArgument(
        "dummy_imu_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "dummy_imu.yaml"]),
    )

    use_lidar = DeclareLaunchArgument(
        "use_lidar", default_value="true", description="LiDAR + FAST-LIO パイプラインを有効にする"
    )
    use_t265_odom = DeclareLaunchArgument(
        "use_t265_odom",
        default_value="false",
        description="T265 ビジュアルオドメトリを /odom の第3 fallback として使用する",
    )
    use_slam = DeclareLaunchArgument("use_slam", default_value="true")
    slam_params = DeclareLaunchArgument(
        "slam_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "slam_toolbox_async.yaml"]),
    )
    slam_node_exec = DeclareLaunchArgument(
        "slam_node_exec", default_value="async_slam_toolbox_node"
    )
    slam_map_frame = DeclareLaunchArgument("slam_map_frame", default_value="map")
    slam_odom_frame = DeclareLaunchArgument("slam_odom_frame", default_value="odom")
    use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")

    spark_node = Node(
        package="spark_fast_lio",
        executable="spark_lio_mapping",
        name="lio_mapping",
        output="screen",
        remappings=[
            ("lidar", LaunchConfiguration("lidar_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
            # /spark_lio/odom に出力し、odom_selector が /odom に中継する
            ("odometry", "/spark_lio/odom"),
        ],
        parameters=[
            LaunchConfiguration("config_path"),
            {
                "common.map_frame": LaunchConfiguration("map_frame"),
                "common.base_frame": LaunchConfiguration("base_frame"),
                "common.lidar_frame": LaunchConfiguration("lidar_frame"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )

    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp",
        output="screen",
        remappings=[
            ("pointcloud_topic", LaunchConfiguration("lidar_topic_raw")),
        ],
        parameters=[
            LaunchConfiguration("kiss_icp_params"),
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "lidar_odom_frame": LaunchConfiguration("map_frame"),
                "publish_odom_tf": False,  # odom_selector が TF を担当
                "invert_odom_tf": False,
            },
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("use_lidar"), "' == 'true' and ", str(kiss_icp_available)]
            )
        ),
        respawn=True,
        respawn_delay=3.0,
    )

    # odom_selector は LiDAR が有効 OR T265 odom が有効な場合に起動する
    odom_selector_active = PythonExpression(
        [
            "'",
            LaunchConfiguration("use_lidar"),
            "' == 'true' or '",
            LaunchConfiguration("use_t265_odom"),
            "' == 'true'",
        ]
    )
    # static fallback TF は LiDAR も T265 も無効な場合のみ
    odom_static_fallback_active = PythonExpression(
        [
            "'",
            LaunchConfiguration("use_lidar"),
            "' != 'true' and '",
            LaunchConfiguration("use_t265_odom"),
            "' != 'true'",
        ]
    )

    odom_selector_node = Node(
        package="bringup",
        executable="odom_selector.py",
        name="odom_selector",
        output="screen",
        parameters=[
            {
                "base_frame": LaunchConfiguration("base_frame"),
                "odom_frame": LaunchConfiguration("map_frame"),
                # use_imu=false 時は IMU health が届かないため、タイムアウトで自動 KISS-ICP へ
                "health_timeout_s": 5.0,
                "allow_kiss_fallback": kiss_icp_available,
                "use_lidar": LaunchConfiguration("use_lidar"),
                "use_t265_odom": LaunchConfiguration("use_t265_odom"),
            }
        ],
        condition=IfCondition(odom_selector_active),
        respawn=True,
        respawn_delay=2.0,
    )

    # LiDAR も T265 も無効な場合のフォールバック: odom→base_link を単位変換で配信
    # ロボットの TF ツリーが途切れないよう最低限の TF を保証する
    odom_base_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_base_fallback_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            LaunchConfiguration("map_frame"),
            "--child-frame-id",
            LaunchConfiguration("base_frame"),
        ],
        output="screen",
        condition=IfCondition(odom_static_fallback_active),
    )

    velodyne_driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver",
        output="screen",
        parameters=[LaunchConfiguration("velodyne_driver_params")],
        condition=IfCondition(LaunchConfiguration("use_velodyne")),
    )

    velodyne_pointcloud_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_pointcloud",
        output="screen",
        parameters=[
            LaunchConfiguration("velodyne_pointcloud_params"),
            {"calibration": LaunchConfiguration("velodyne_calibration")},
        ],
        condition=IfCondition(LaunchConfiguration("use_velodyne")),
    )

    scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", LaunchConfiguration("lidar_topic_raw")),
            ("scan", LaunchConfiguration("scan_topic")),
        ],
        parameters=[LaunchConfiguration("scan_params")],
        condition=IfCondition(LaunchConfiguration("use_scan")),
    )

    time_fix_node = Node(
        package="bringup",
        executable="fix_pointcloud_time_node.py",
        name="fix_pointcloud_time",
        output="screen",
        parameters=[
            LaunchConfiguration("time_fix_params"),
            {
                "input_topic": LaunchConfiguration("lidar_topic_raw"),
                "output_topic": LaunchConfiguration("lidar_topic"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_time_fix")),
    )

    dummy_imu_node = Node(
        package="bringup",
        executable="dummy_imu_node.py",
        name="dummy_imu",
        output="screen",
        parameters=[
            LaunchConfiguration("dummy_imu_params"),
            {"topic": LaunchConfiguration("imu_topic")},
        ],
        condition=IfCondition(LaunchConfiguration("use_dummy_imu")),
    )

    slam_node = Node(
        package="slam_toolbox",
        executable=LaunchConfiguration("slam_node_exec"),
        name="slam_toolbox",
        output="screen",
        parameters=[
            LaunchConfiguration("slam_params"),
            {
                "map_frame": LaunchConfiguration("slam_map_frame"),
                "odom_frame": LaunchConfiguration("slam_odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("use_slam")),
        respawn=True,
        respawn_delay=5.0,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            *(
                [
                    LogInfo(
                        msg=(
                            "[WARN][perception.launch] libtbb.so.12 is missing; "
                            "disabling KISS-ICP and keeping spark_fast_lio odom"
                        )
                    )
                ]
                if not kiss_icp_available
                else []
            ),
            use_lidar,
            use_t265_odom,
            config_path,
            lidar_topic_raw,
            lidar_topic,
            imu_topic,
            map_frame,
            base_frame,
            lidar_frame,
            kiss_icp_params,
            use_velodyne,
            velodyne_driver_params,
            velodyne_pointcloud_params,
            velodyne_calibration,
            use_scan,
            scan_topic,
            scan_params,
            use_time_fix,
            time_fix_params,
            use_dummy_imu,
            dummy_imu_params,
            use_slam,
            slam_params,
            slam_node_exec,
            slam_map_frame,
            slam_odom_frame,
            use_rviz,
            velodyne_driver_node,
            velodyne_pointcloud_node,
            scan_node,
            time_fix_node,
            dummy_imu_node,
            spark_node,
            kiss_icp_node,
            odom_selector_node,
            odom_base_static_tf,
            slam_node,
            rviz_node,
        ]
    )
