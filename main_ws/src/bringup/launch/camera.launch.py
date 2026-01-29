import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # configファイルのパスを取得
    config = os.path.join(
        get_package_share_directory('robot_bringup'),
        'config',
        'usb_cam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            namespace='camera', # トピック名が /camera/image_raw になる
            parameters=[config],
            output='screen',
        )
    ])
