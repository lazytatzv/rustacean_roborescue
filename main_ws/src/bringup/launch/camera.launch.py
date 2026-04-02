import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _load_cameras(bringup_share: str) -> list:
    path = os.path.join(bringup_share, "config", "cameras.yaml")
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
        return data.get("cameras", [])
    except Exception as e:
        print(f"[camera.launch] WARNING: failed to load cameras.yaml: {e}")
        return []


def _camera_available(cam: dict) -> tuple[bool, str]:
    if not bool(cam.get("enabled", True)):
        return False, f"{cam.get('name', 'unknown')}: disabled by config"

    driver = cam.get("driver", "v4l2")
    if driver in ("v4l2", "theta_s"):
        dev = cam.get("device", "")
        if not dev or not os.path.exists(dev):
            return False, f"{cam.get('name', 'unknown')}: missing video device {dev}"

    if driver == "realsense":
        try:
            get_package_share_directory("realsense2_camera")
        except PackageNotFoundError:
            return False, f"{cam.get('name', 'unknown')}: realsense2_camera package not installed"

    return True, ""


def _video_device_realpath_key(dev_path: str) -> str:
    """同一物理カメラの重複起動を避けるための識別キーを返す。"""
    base = os.path.basename(dev_path)
    if not base.startswith("video"):
        return ""
    sysfs_dev = os.path.join("/sys/class/video4linux", base, "device")
    if not os.path.exists(sysfs_dev):
        return ""
    try:
        return os.path.realpath(sysfs_dev)
    except Exception:
        return ""


def _v4l2_nodes(cam: dict, qr_model_dir: str, ns: str) -> list:
    """v4l2 ドライバのコンポーザブルノードリストを返す。"""
    composable = [
        ComposableNode(
            package="v4l2_camera",
            plugin="v4l2_camera::V4L2Camera",
            name="v4l2_camera",
            namespace=ns,
            parameters=[
                {
                    "video_device": cam.get("device", "/dev/video0"),
                    "pixel_format": cam.get("pixel_format", "YUYV"),
                    "image_size": [cam.get("width", 640), cam.get("height", 480)],
                    "time_per_frame": [1, cam.get("fps", 30)],
                    "camera_frame_id": cam.get("frame_id", f"{cam['name']}_camera_link"),
                    "image_raw.ffmpeg.encoder": cam.get("encoder", "libx264"),
                    "image_raw.ffmpeg.bit_rate": cam.get("bitrate", 2000000),
                    "image_raw.ffmpeg.gop_size": 10,
                    "image_raw.ffmpeg.av_options": "profile:baseline,preset:ultrafast,x264-params:repeat_headers=1",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]
    return composable


def _theta_s_nodes(cam: dict, ns: str) -> list:
    """RICOH THETA S (UVC live streaming) のコンポーザブルノードリストを返す。
    THETA S は USB接続時にMJPEGのUVCデバイスとして認識される。
    """
    return [
        ComposableNode(
            package="v4l2_camera",
            plugin="v4l2_camera::V4L2Camera",
            name="v4l2_camera",
            namespace=ns,
            parameters=[
                {
                    "video_device": cam.get("device", "/dev/video4"),
                    "pixel_format": "MJPG",
                    "image_size": [cam.get("width", 1280), cam.get("height", 720)],
                    "time_per_frame": [1, cam.get("fps", 15)],
                    "camera_frame_id": cam.get("frame_id", f"{cam['name']}_camera_link"),
                    "image_raw.ffmpeg.encoder": cam.get("encoder", "libx264"),
                    "image_raw.ffmpeg.bit_rate": cam.get("bitrate", 4000000),
                    "image_raw.ffmpeg.gop_size": 10,
                    "image_raw.ffmpeg.av_options": "profile:baseline,preset:ultrafast,x264-params:repeat_headers=1",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def _realsense_nodes(cam: dict, ns: str) -> list:
    """RealSense ドライバのコンポーザブルノードリストを返す。"""
    params = {
        "serial_no": cam.get("serial", ""),
        "enable_color": cam.get("enable_color", True),
        "enable_depth": cam.get("enable_depth", False),
        "enable_pose": cam.get("enable_pose", False),
        "color_width": cam.get("color_width", 640),
        "color_height": cam.get("color_height", 480),
        "color_fps": cam.get("color_fps", 30),
        "depth_width": cam.get("depth_width", 640),
        "depth_height": cam.get("depth_height", 480),
        "depth_fps": cam.get("depth_fps", 30),
    }
    return [
        ComposableNode(
            package="realsense2_camera",
            plugin="realsense2_camera::RealSenseNodeFactory",
            name="realsense2_camera",
            namespace=ns,
            parameters=[params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def _qr_node(cam: dict, qr_model_dir: str, ns: str, image_topic: str) -> ComposableNode:
    return ComposableNode(
        package="qr_detector_cpp",
        plugin="qr_detector_cpp::QrDetectorNode",
        name="qr_detector",
        namespace=ns,
        remappings=[("image_raw", image_topic)],
        parameters=[
            {
                "model_dir": qr_model_dir,
                "publish_compressed": False,
                "detection_interval": 1,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def _make_camera_group(cam: dict, qr_model_dir: str, use_camera_cfg: LaunchConfiguration) -> list:
    name = cam["name"]
    ns = f"camera_{name}"
    driver = cam.get("driver", "v4l2")
    use_qr = cam.get("use_qr", False)

    # ドライバごとのデフォルトimage_topic
    default_image_topic = "color/image_raw" if driver == "realsense" else "image_raw"
    image_topic = cam.get("image_topic", default_image_topic)

    # コンポーザブルノード
    if driver == "realsense":
        composable_nodes = _realsense_nodes(cam, ns)
    elif driver == "theta_s":
        composable_nodes = _theta_s_nodes(cam, ns)
    else:
        composable_nodes = _v4l2_nodes(cam, qr_model_dir, ns)

    if use_qr:
        composable_nodes.append(_qr_node(cam, qr_model_dir, ns, image_topic))

    container = ComposableNodeContainer(
        name=f"camera_{name}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
        condition=IfCondition(use_camera_cfg),
        respawn=False,
        respawn_delay=3.0,
    )

    # ffmpeg ブリッジ: カラー画像があるカメラのみ
    has_color = driver == "v4l2" or cam.get("enable_color", True)
    actions = [container]
    if has_color:
        ffmpeg_in = f"/{ns}/{image_topic}/ffmpeg"
        ffmpeg_out = f"/{ns}/compressed_video"
        actions.append(
            Node(
                package="bringup",
                executable="ffmpeg_to_foxglove_video.py",
                name=f"ffmpeg_to_foxglove_{name}",
                parameters=[
                    {
                        "use_sim_time": False,
                        "input_topic": ffmpeg_in,
                        "output_topic": ffmpeg_out,
                    }
                ],
                condition=IfCondition(use_camera_cfg),
                respawn=False,
                respawn_delay=3.0,
            )
        )

        # Backward-compatible single-topic output for operator dashboards.
        # Keep per-camera topics, and additionally expose front camera to a
        # stable global topic that Foxglove layouts can subscribe to.
        if name == "front":
            actions.append(
                Node(
                    package="bringup",
                    executable="ffmpeg_to_foxglove_video.py",
                    name="ffmpeg_to_foxglove_front_compat",
                    parameters=[
                        {
                            "use_sim_time": False,
                            "input_topic": ffmpeg_in,
                            "output_topic": "/camera/image_raw/foxglove_video",
                        }
                    ],
                    condition=IfCondition(use_camera_cfg),
                    respawn=False,
                    respawn_delay=3.0,
                )
            )

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    qr_share = get_package_share_directory("qr_detector")
    qr_model_dir = os.path.join(qr_share, "models")

    arg_use_camera = DeclareLaunchArgument(
        "use_camera", default_value="true", description="カメラ + QR 検出を有効にする"
    )
    use_camera_cfg = LaunchConfiguration("use_camera")

    cameras = _load_cameras(bringup_share)
    if not cameras:
        print("[camera.launch] WARNING: no cameras defined in cameras.yaml")

    actions = [arg_use_camera]
    seen_v4l2_phys = set()
    for cam in cameras:
        available, reason = _camera_available(cam)
        if not available:
            actions.append(LogInfo(msg=f"[WARN][camera.launch] skip camera: {reason}"))
            continue

        driver = cam.get("driver", "v4l2")
        if driver in ("v4l2", "theta_s"):
            dev = cam.get("device", "")
            key = _video_device_realpath_key(dev)
            if key:
                if key in seen_v4l2_phys:
                    actions.append(
                        LogInfo(
                            msg=(
                                "[WARN][camera.launch] skip camera: "
                                f"{cam.get('name', 'unknown')}: duplicate physical v4l2 device {dev}"
                            )
                        )
                    )
                    continue
                seen_v4l2_phys.add(key)

        actions.extend(_make_camera_group(cam, qr_model_dir, use_camera_cfg))

    return LaunchDescription(actions)
