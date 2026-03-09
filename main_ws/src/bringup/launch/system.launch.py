# ここから他のlaunchファイルを呼び出し、
# このスクリプトでロボットのシステムを起動する

# joy_nodeはロボット側ではなくoperator側で、特に複雑な設定もいらないので、
# 今のところはros2 run joy joy_nodeで起動する方向性で考え中

# systemdに便り過ぎるのもまた複雑性が増し、
# Justfileで頑張るのもまた大変なので、なるべくlaunchから
# zenohdもlaunchから起動する

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    # 各サブLaunchファイルのパス
    network_launch = os.path.join(bringup_dir, 'launch', 'network.launch.py')
    perception_launch = os.path.join(bringup_dir, 'launch', 'perception.launch.py')
    control_launch = os.path.join(bringup_dir, 'launch', 'control.launch.py')
    nav2_launch = os.path.join(bringup_dir, 'launch', 'nav2.launch.py')

    use_nav2 = DeclareLaunchArgument('use_nav2', default_value='false',
                                      description='Nav2 自律走行を有効にする')

    return LaunchDescription([
        use_nav2,

        # 1. ネットワーク（Zenoh）の起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(network_launch),
        ),

        # 2. 認識・SLAM系の起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={'use_slam': 'true', 'use_rviz': 'false'}.items()
        ),

        # 3. 制御系の起動
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
        ),

        # 4. Nav2 自律走行（オプション）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(LaunchConfiguration('use_nav2')),
        ),
    ])
