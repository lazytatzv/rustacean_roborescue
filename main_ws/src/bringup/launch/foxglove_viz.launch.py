import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    
    # ── URDF Load ──
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(bringup_dir, 'urdf', 'robot.urdf.xacro')]),
        value_type=str
    )

    # ── Nodes ──
    
    # Robot State Publisher (TF発行 & URDF配信)
    # これが重要: /tf と /robot_description を Foxglove に送る
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    # Foxglove Bridge (WebSocket経由で接続する場合に必要)
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge'
    )

    return LaunchDescription([
        robot_state_publisher,
        foxglove_bridge
    ])
