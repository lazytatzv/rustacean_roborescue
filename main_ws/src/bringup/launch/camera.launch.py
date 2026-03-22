import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    qr_share = get_package_share_directory("qr_detector")

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
                # ffmpeg_image_transport params (published as /camera/image_raw/ffmpeg)
                "ffmpeg_image_transport.encoding": "h264",
                # Foxglove-compatible encoder settings:
                # - use BASELINE (no B-frames)
                # - low-latency preset/tune
                # - enforce no B-frames via x264 opts and set an explicit bitrate
                "ffmpeg_image_transport.profile": "baseline",
                "ffmpeg_image_transport.preset": "ultrafast",
                "ffmpeg_image_transport.tune": "zerolatency",
                "ffmpeg_image_transport.x264opts": "bframes=0",
                "ffmpeg_image_transport.bitrate": 2000000,  # 2 Mbps default; tune as needed
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
    )

    return LaunchDescription([container])
