"""Nav2 自律走行 Launch ファイル.

slam_toolbox の /map + spark_fast_lio の odom→base_link TF を前提に、
Nav2 スタックを起動する。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("bringup")

    nav2_params = DeclareLaunchArgument(
        "nav2_params",
        default_value=PathJoinSubstitution([bringup_share, "config", "nav2_params.yaml"]),
        description="Nav2 パラメータ YAML のパス",
    )
    autostart = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="lifecycle_manager が自動で activate する",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="シミュレーション時は true",
    )

    params_file = LaunchConfiguration("nav2_params")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Nav2 ノード群 ──
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "cmd_vel_nav")],  # velocity_smoother 経由
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "cmd_vel_nav"),       # controller_server → smoother
            ("cmd_vel_smoothed", "cmd_vel"),   # smoother → crawler_driver
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": LaunchConfiguration("autostart"),
            "node_names": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "velocity_smoother",
            ],
        }],
    )

    return LaunchDescription([
        nav2_params,
        autostart,
        use_sim_time_arg,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_manager,
    ])
