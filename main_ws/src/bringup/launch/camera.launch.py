import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    qr_share = get_package_share_directory("qr_detector")

    arg_use_camera = DeclareLaunchArgument(
        "use_camera", default_value="true", description="カメラ + QR 検出を有効にする"
    )

    # v4l2_camera configuration
    # Note: Use /dev/video0 or similar.
    camera_node = ComposableNode(
        package="v4l2_camera",
        plugin="v4l2_camera::V4L2Camera",
        name="v4l2_camera",
        namespace="camera",
        parameters=[
            {
                "video_device": "/dev/video0",
                "image_size": [640, 480],
                "time_per_frame": [1, 30],
                "camera_frame_id": "camera_link",
                # ffmpeg_image_transport パラメータ: <topic>.<transport>.<param>
                # topic = "image_raw"、namespace は除く
                "image_raw.ffmpeg.encoder": "libx264",
                "image_raw.ffmpeg.bit_rate": 2000000,  # 2 Mbps
                "image_raw.ffmpeg.gop_size": 10,  # 10フレームごとにキーフレーム
                # Foxglove CompressedVideo 要件:
                # - profile=baseline: B-framesなし (Foxglove非対応のため必須)
                # - x264-params:repeat_headers=1: IDRごとにSPS/PPS付与
                "image_raw.ffmpeg.av_options": "profile:baseline,preset:ultrafast,x264-params:repeat_headers=1",
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    qr_detector_node = ComposableNode(
        package="qr_detector_cpp",
        plugin="qr_detector_cpp::QrDetectorNode",
        name="qr_detector",
        namespace="camera",
        remappings=[
            ("/camera/image_raw", "/camera/image_raw"),
        ],
        parameters=[
            {
                "model_dir": os.path.join(qr_share, "models"),
                "publish_compressed": False,  # We use ffmpeg transport for robot stream
                "detection_interval": 1,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # Multi-threaded for parallelism
        composable_node_descriptions=[
            camera_node,
            qr_detector_node,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    # FFMPEGPacket → foxglove_msgs/CompressedVideo 変換
    # foxglove_bridge + Foxglove Studio でネイティブH264表示するために必要
    ffmpeg_to_foxglove = Node(
        package="bringup",
        executable="ffmpeg_to_foxglove_video.py",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    return LaunchDescription([arg_use_camera, container, ffmpeg_to_foxglove])
