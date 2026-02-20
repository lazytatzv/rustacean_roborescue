from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")

    config_path = DeclareLaunchArgument(
        "config_path",
        default_value=PathJoinSubstitution([bringup_share, "config", "spark_fast_lio_min.yaml"]),
    )
    lidar_topic_raw = DeclareLaunchArgument("lidar_topic_raw", default_value="/velodyne_points")
    lidar_topic = DeclareLaunchArgument("lidar_topic", default_value="/velodyne_points_fixed")
    imu_topic = DeclareLaunchArgument("imu_topic", default_value="/imu/data")
    map_frame = DeclareLaunchArgument("map_frame", default_value="odom")
    base_frame = DeclareLaunchArgument("base_frame", default_value="base_link")
    lidar_frame = DeclareLaunchArgument("lidar_frame", default_value="velodyne")
    lidar_tf_x = DeclareLaunchArgument("lidar_tf_x", default_value="0.0")
    lidar_tf_y = DeclareLaunchArgument("lidar_tf_y", default_value="0.0")
    lidar_tf_z = DeclareLaunchArgument("lidar_tf_z", default_value="0.0")
    lidar_tf_roll = DeclareLaunchArgument("lidar_tf_roll", default_value="0.0")
    lidar_tf_pitch = DeclareLaunchArgument("lidar_tf_pitch", default_value="0.0")
    lidar_tf_yaw = DeclareLaunchArgument("lidar_tf_yaw", default_value="0.0")

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
        default_value=PathJoinSubstitution([bringup_share, "config", "pointcloud_to_laserscan.yaml"]),
    )

    use_time_fix = DeclareLaunchArgument("use_time_fix", default_value="true")
    time_fix_params = DeclareLaunchArgument(
        "time_fix_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "fix_pointcloud_time.yaml"]),
    )

    use_dummy_imu = DeclareLaunchArgument("use_dummy_imu", default_value="false")
    dummy_imu_params = DeclareLaunchArgument(
        "dummy_imu_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "dummy_imu.yaml"]),
    )

    use_slam = DeclareLaunchArgument("use_slam", default_value="true")
    slam_params = DeclareLaunchArgument(
        "slam_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "slam_toolbox_async.yaml"]),
    )
    slam_node_exec = DeclareLaunchArgument("slam_node_exec", default_value="async_slam_toolbox_node")
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
        ],
        parameters=[
            LaunchConfiguration("config_path"),
            {
                "common.map_frame": LaunchConfiguration("map_frame"),
                "common.base_frame": LaunchConfiguration("base_frame"),
                "common.lidar_frame": LaunchConfiguration("lidar_frame"),
            },
        ],
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        arguments=[
            LaunchConfiguration("lidar_tf_x"),
            LaunchConfiguration("lidar_tf_y"),
            LaunchConfiguration("lidar_tf_z"),
            LaunchConfiguration("lidar_tf_roll"),
            LaunchConfiguration("lidar_tf_pitch"),
            LaunchConfiguration("lidar_tf_yaw"),
            LaunchConfiguration("base_frame"),
            LaunchConfiguration("lidar_frame"),
        ],
        output="screen",
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
            config_path,
            lidar_topic_raw,
            lidar_topic,
            imu_topic,
            map_frame,
            base_frame,
            lidar_frame,
            lidar_tf_x,
            lidar_tf_y,
            lidar_tf_z,
            lidar_tf_roll,
            lidar_tf_pitch,
            lidar_tf_yaw,
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
            lidar_static_tf,
            slam_node,
            rviz_node,
        ]
    )
