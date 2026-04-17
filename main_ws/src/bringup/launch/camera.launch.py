import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def _v4l2_nodes(cam: dict, ns: str) -> list:
    """v4l2 ドライバのコンポーザブルノードを返す。JPEG 配信は別 Node で行う。"""
    return [
        ComposableNode(
            package="v4l2_camera",
            plugin="v4l2_camera::V4L2Camera",
            name="v4l2_camera",
            namespace=ns,
            parameters=[
                {
                    "video_device": cam.get("device", "/dev/video0"),
                    "pixel_format": cam.get("pixel_format", "MJPG"),
                    "output_encoding": cam.get("output_encoding", "rgb8"),
                    "image_size": [cam.get("width", 640), cam.get("height", 480)],
                    "time_per_frame": [1, cam.get("fps", 30)],
                    "camera_frame_id": cam.get("frame_id", f"{cam['name']}_camera_link"),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def _theta_s_nodes(cam: dict, ns: str) -> list:
    """RICOH THETA S (UVC live streaming) のコンポーザブルノードを返す。

    RICOH THETA S は MJPEG のみ出力する。v4l2_camera は MJPEG 非対応のため
    GStreamer ベースの gscam を使用する。jpegdec で確実にデコードする。

    GStreamer framerate caps は fps (= fps_denominator/fps_numerator) で指定する。
    cameras.yaml の fps_numerator/fps_denominator は time_per_frame (1/fps の分子/分母) なので
    GStreamer 向けには逆数 (fps_denominator/fps_numerator) を使う。
    """
    fps_num = cam.get("fps_numerator", 200)     # time_per_frame 分子 (= 1/fps の分子)
    fps_den = cam.get("fps_denominator", 2997)  # time_per_frame 分母
    device = cam.get("device", "/dev/video4")
    width = cam.get("width", 1280)
    height = cam.get("height", 720)

    # framerate = fps = fps_den/fps_num = 2997/200 ≒ 14.985 fps
    pipeline = (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={width},height={height},framerate={fps_den}/{fps_num} ! "
        f"jpegdec ! videoconvert ! video/x-raw,format=RGB"
    )

    return [
        ComposableNode(
            package="gscam",
            plugin="gscam::GSCam",
            name="gscam",
            namespace=ns,
            remappings=[
                # gscam は camera/image_raw に配信するが、
                # 他ノード (QR検出・republish) は image_raw を期待するため合わせる
                ("camera/image_raw", "image_raw"),
                ("camera/camera_info", "camera_info"),
            ],
            parameters=[
                {
                    "gscam_config": pipeline,
                    "image_encoding": cam.get("output_encoding", "rgb8"),
                    "camera_name": ns,
                    "frame_id": cam.get("frame_id", f"{cam['name']}_camera_link"),
                    "sync_sink": False,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def _realsense_nodes(cam: dict, ns: str) -> list:
    """RealSense ドライバのコンポーザブルノードを返す。"""
    params = {
        "serial_no": cam.get("serial", ""),
        "base_frame_id": f"{ns}_link",
        "enable_color": cam.get("enable_color", True),
        "enable_depth": cam.get("enable_depth", False),
        "enable_pose": cam.get("enable_pose", False),
        "enable_accel": cam.get("enable_accel", False),
        "enable_gyro": cam.get("enable_gyro", False),
        "enable_pointcloud": cam.get("enable_pointcloud", False),
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


def _compressed_republish_node(
    cam: dict, ns: str, image_topic: str, condition=None
) -> Node:
    """image_transport republish で compressed を配信するスタンドアロンノード。

    image_transport::RepublishNode composable plugin が nix 環境で未登録のため
    standalone Node として起動する。subscriber がいないときも常時動作するが、
    compressed transport 自体が lazy なので帯域消費は限定的。
    """
    return Node(
        package="image_transport",
        executable="republish",
        name="republish_compressed",
        namespace=ns,
        arguments=["raw"],
        remappings=[
            ("in", image_topic),
            ("out", image_topic),
        ],
        parameters=[
            {
                "in_transport": "raw",
                "out_transport": "compressed",
                "compressed.jpeg_quality": cam.get("jpeg_quality", 80),
            }
        ],
        condition=condition,
        respawn=True,
        respawn_delay=3.0,
    )


def _qr_node_standalone(cam: dict, qr_model_dir: str, ns: str, image_topic: str, condition=None) -> Node:
    """QR検出をスタンドアロンNodeとして起動。クラッシュ時にカメラに影響しない。"""
    return Node(
        package="qr_detector_cpp",
        executable="qr_detector_node",
        name="qr_detector",
        namespace=ns,
        remappings=[("image_raw", image_topic)],
        parameters=[
            {
                "model_dir": qr_model_dir,
                "publish_compressed": False,
                "detection_interval": 30,
            }
        ],
        condition=condition,
        respawn=True,
        respawn_delay=3.0,
    )


def _qr_node_composable(cam: dict, qr_model_dir: str, ns: str, image_topic: str) -> ComposableNode:
    """QR検出をComposableNodeとして返す。intra-process通信でパフォーマンス向上。"""
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
                "detection_interval": 30,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )


def _make_camera_group(
    cam: dict,
    qr_model_dir: str,
    use_camera_cfg: LaunchConfiguration,
    use_depth_guard_cfg: LaunchConfiguration,
) -> list:
    name = cam["name"]
    ns = f"camera_{name}"
    driver = cam.get("driver", "v4l2")
    use_qr = cam.get("use_qr", False)

    # ドライバごとのデフォルト raw image トピック
    default_image_topic = "color/image_raw" if driver == "realsense" else "image_raw"
    image_topic = cam.get("image_topic", default_image_topic)

    # コンポーザブルノード (ドライバのみ、QR は別コンテナに分離)
    if driver == "realsense":
        composable_nodes = _realsense_nodes(cam, ns)
    elif driver == "theta_s":
        composable_nodes = _theta_s_nodes(cam, ns)
    else:
        composable_nodes = _v4l2_nodes(cam, ns)

    has_color = driver != "realsense" or cam.get("enable_color", True)

    container = ComposableNodeContainer(
        name=f"camera_{name}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
        condition=IfCondition(use_camera_cfg),
        respawn=True,
        respawn_delay=3.0,
    )

    actions = [container]

    # QR 検出ノード起動
    # qr_use_component: true  → カメラと同一コンテナ (intra-process通信、高パフォーマンス)
    # qr_use_component: false → 別プロセス (クラッシュ分離、デフォルト)
    if use_qr:
        if qr_model_dir:
            qr_use_component = cam.get("qr_use_component", False)
            if qr_use_component:
                # カメラコンテナに追加（既に作成済みのcontainerには追加できないので再構築）
                composable_nodes.append(_qr_node_composable(cam, qr_model_dir, ns, image_topic))
                # containerを更新（actionsの最初の要素を置き換え）
                actions[0] = ComposableNodeContainer(
                    name=f"camera_{name}_container",
                    namespace="",
                    package="rclcpp_components",
                    executable="component_container_mt",
                    composable_node_descriptions=composable_nodes,
                    output="screen",
                    condition=IfCondition(use_camera_cfg),
                    respawn=True,
                    respawn_delay=3.0,
                )
            else:
                actions.append(
                    _qr_node_standalone(cam, qr_model_dir, ns, image_topic, condition=IfCondition(use_camera_cfg))
                )
        else:
            print(
                f"[camera.launch] WARNING: use_qr=true for {name} "
                "but qr_detector package not found, skipping QR"
            )

    # image_transport republish (standalone): compressed トピックを配信する。
    # composable plugin が nix 環境で未登録のためコンテナ外で起動する。
    # realsense は RGB センサー初期化が遅いため、zenoh ピア間サブスクリプションが
    # 確立される前に republish が起動してしまう。delay で待機させる。
    if has_color:
        republish_delay = 5.0 if driver == "realsense" else 2.0
        actions.append(
            TimerAction(
                period=republish_delay,
                actions=[
                    _compressed_republish_node(
                        cam, ns, image_topic, condition=IfCondition(use_camera_cfg)
                    )
                ],
            )
        )

    # depthimage_to_laserscan: 深度画像 → LaserScan → nav2 costmap
    if driver == "realsense" and cam.get("enable_depth", False):
        try:
            get_package_share_directory("depthimage_to_laserscan")
            actions.append(
                Node(
                    package="depthimage_to_laserscan",
                    executable="depthimage_to_laserscan_node",
                    name=f"depth_to_scan_{name}",
                    remappings=[
                        ("image", f"/{ns}/depth/image_rect_raw"),
                        ("camera_info", f"/{ns}/depth/camera_info"),
                        ("scan", "/scan_depth"),
                    ],
                    parameters=[
                        {
                            "scan_height": 5,
                            "scan_time": 0.033,
                            "range_min": 0.1,
                            "range_max": 3.0,
                        }
                    ],
                    condition=IfCondition(use_camera_cfg),
                    respawn=True,
                    respawn_delay=5.0,
                )
            )
        except PackageNotFoundError:
            print(
                f"[camera.launch] WARNING: depthimage_to_laserscan not found, "
                f"skipping depth→laserscan for {name}"
            )

    # depth_guard: オペレーター向け近接障害物警告
    if driver == "realsense" and cam.get("enable_depth", False):
        actions.append(
            Node(
                package="depth_guard",
                executable="depth_guard",
                name=f"depth_guard_{name}",
                output="screen",
                parameters=[
                    {
                        "enabled": True,
                        "depth_topic": f"/{ns}/depth/image_rect_raw",
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            use_camera_cfg,
                            "' == 'true' and '",
                            use_depth_guard_cfg,
                            "' == 'true'",
                        ]
                    )
                ),
                respawn=True,
                respawn_delay=5.0,
            )
        )

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    try:
        qr_share = get_package_share_directory("qr_detector")
        qr_model_dir = os.path.join(qr_share, "models")
    except PackageNotFoundError:
        print("[camera.launch] WARNING: qr_detector package not found, QR detection disabled")
        qr_model_dir = ""

    arg_use_camera = DeclareLaunchArgument(
        "use_camera", default_value="true", description="カメラ + QR 検出を有効にする"
    )
    arg_use_depth_guard = DeclareLaunchArgument(
        "use_depth_guard",
        default_value="false",
        description="D435i 深度近接検出ノード (depth_guard) を有効にする",
    )
    use_camera_cfg = LaunchConfiguration("use_camera")
    use_depth_guard_cfg = LaunchConfiguration("use_depth_guard")

    cameras = _load_cameras(bringup_share)
    if not cameras:
        print("[camera.launch] WARNING: no cameras defined in cameras.yaml")

    actions = [arg_use_camera, arg_use_depth_guard]
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

        actions.extend(
            _make_camera_group(cam, qr_model_dir, use_camera_cfg, use_depth_guard_cfg)
        )

    return LaunchDescription(actions)
