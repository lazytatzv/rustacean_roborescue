import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(bringup_dir, 'urdf', 'robot.urdf.xacro'),
                 ' use_real_hw:=false']),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            # publish_frequency>0 で fixed joints も /tf に定期再送 (zenoh transient_local 非対応の回避)
            parameters=[{'robot_description': robot_description,
                         'use_tf_static': False}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'zeros': {
                'flipper_fl_joint': -1.5708,
                'flipper_fr_joint': 1.5708,
                'flipper_bl_joint': -1.5708,
                'flipper_br_joint': 1.5708,
            }}]
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
        )
    ])
