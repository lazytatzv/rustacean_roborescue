# ここから他のlaunchファイルを呼び出し、
# このスクリプトでロボットのシステムを起動する

# operator側でfoxglove_bridgeとjoy_nodeを立ち上げる

# systemdに便り過ぎるのもまた複雑性が増し、
# Justfileで頑張るのもまた大変なので、なるべくlaunchから
# zenohdもlaunchから起動する

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    # 各サブLaunchファイルのパス
    network_launch = os.path.join(bringup_dir, 'launch', 'network.launch.py')
    perception_launch = os.path.join(bringup_dir, 'launch', 'perception.launch.py')
    control_launch = os.path.join(bringup_dir, 'launch', 'control.launch.py')
    camera_launch = os.path.join(bringup_dir, 'launch', 'camera.launch.py')
    nav2_launch = os.path.join(bringup_dir, 'launch', 'nav2.launch.py')

    use_nav2 = DeclareLaunchArgument('use_nav2', default_value='false',
                                      description='Nav2 自律走行を有効にする')

    # ── Robot State Publisher (URDF → TF: base_link→各センサ/アーム/フリッパ) ──
    urdf_file = os.path.join(bringup_dir, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str,
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
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

    return LaunchDescription([
        use_nav2,

        # 0. Robot State Publisher (URDF TF: base_link→各センサ/アーム/フリッパ)
        robot_state_publisher_node,

        # 1. Foxglove Bridge (WebSocket :8765)
        #foxglove_bridge_node,

        # ネットワーク（Zenoh）の起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(network_launch),
        ),

        # 認識・SLAM系の起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={'use_slam': 'true', 'use_rviz': 'false'}.items()
        ),

        # カメラの起動 (qr_detector が /camera/image_raw を必要とする)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
        ),

        # 制御系の起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
        ),

        # Nav2 自律走行（オプション）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
    ])
