import math
import struct
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

try:
    from PIL import Image, ImageDraw

    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False


class RoboMapExporter(Node):
    def __init__(self):
        super().__init__("robo_map_exporter")

        self.declare_parameter("output_dir", "/tmp/robocup_outputs")
        self.declare_parameter("team_name", "RustaceanRescue")
        self.declare_parameter("country", "Japan")
        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("export_interval", 60.0)
        self.declare_parameter("cloud_topic", "/cloud_map")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")

        self._output_dir = Path(self.get_parameter("output_dir").get_parameter_value().string_value)
        self._team_name = self.get_parameter("team_name").get_parameter_value().string_value
        self._country = self.get_parameter("country").get_parameter_value().string_value
        self._robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        export_interval = float(
            self.get_parameter("export_interval").get_parameter_value().double_value
        )
        cloud_topic = self.get_parameter("cloud_topic").get_parameter_value().string_value
        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._latest_cloud: PointCloud2 | None = None
        self._latest_map: OccupancyGrid | None = None
        self._path_positions: list[tuple[float, float, float]] = []
        self._last_odom_time: float = 0.0

        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(PointCloud2, cloud_topic, self._cloud_cb, sensor_qos)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)

        self.create_service(Trigger, "~/export_now", self._export_srv_cb)

        self.create_timer(export_interval, self._auto_export_cb)

        self.get_logger().info("robo_map_exporter node started")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _odom_cb(self, msg: Odometry) -> None:
        now = time.monotonic()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        if now - self._last_odom_time < 1.0:
            return

        if self._path_positions:
            px, py, pz = self._path_positions[-1]
            dist = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
            if dist < 0.1:
                return

        self._path_positions.append((x, y, z))
        self._last_odom_time = now

    def _export_srv_cb(self, request, response):
        paths = self._do_export()
        response.success = True
        response.message = f"Exported: {', '.join(paths)}"
        return response

    def _auto_export_cb(self) -> None:
        self._do_export()

    def _do_export(self) -> list[str]:
        ts = time.strftime("%Y%m%d_%H%M%S")
        exported = []

        ply_path = self._export_ply(ts)
        if ply_path:
            exported.append(ply_path)

        map_path = self._export_map(ts)
        if map_path:
            exported.append(map_path)

        csv_path = self._export_csv_stub(ts)
        if csv_path:
            exported.append(csv_path)

        if exported:
            self.get_logger().info(f"Exported: {exported}")
        else:
            self.get_logger().info("Export triggered but no data available yet")

        return exported

    def _export_ply(self, ts: str) -> str | None:
        if self._latest_cloud is None:
            return None

        msg = self._latest_cloud
        points = _unpack_pointcloud2(msg)
        if not points:
            return None

        ply_path = self._output_dir / f"{self._team_name}_cloud_{ts}.ply"
        with open(ply_path, "w") as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(points)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            for x, y, z in points:
                f.write(f"{x} {y} {z}\n")

        return str(ply_path)

    def _export_map(self, ts: str) -> str | None:
        if self._latest_map is None or not _PIL_AVAILABLE:
            if not _PIL_AVAILABLE:
                self.get_logger().warn("Pillow not available, skipping map PNG export")
            return None

        grid = self._latest_map
        w = grid.info.width
        h = grid.info.height
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        img = Image.new("RGB", (w, h), (255, 255, 255))
        draw = ImageDraw.Draw(img)

        # Fill cells
        for row in range(h):
            for col in range(w):
                val = grid.data[row * w + col]
                px = col
                # OccupancyGrid row 0 is bottom of map, PIL row 0 is top
                py = h - 1 - row

                if val == -1:
                    tile = (col // 4 + row // 4) % 2
                    color = (0x80, 0x80, 0x80) if tile == 0 else (0xA0, 0xA0, 0xA0)
                    img.putpixel((px, py), color)
                elif val == 0:
                    img.putpixel((px, py), (0xFF, 0xFF, 0xFF))
                elif val >= 65:
                    img.putpixel((px, py), (0x00, 0x00, 0x8B))

        # Grid lines on free cells every 10 pixels
        for col in range(0, w, 10):
            for row in range(h):
                py = h - 1 - row
                val = grid.data[row * w + col]
                if val == 0:
                    img.putpixel((col, py), (0x40, 0x40, 0x40))
        for row in range(0, h, 10):
            py = h - 1 - row
            for col in range(w):
                val = grid.data[row * w + col]
                if val == 0:
                    img.putpixel((col, py), (0x40, 0x40, 0x40))

        # Robot path
        if len(self._path_positions) >= 1:

            def world_to_px(wx, wy):
                col = int((wx - ox) / res)
                row = int((wy - oy) / res)
                return col, h - 1 - row

            path_px = [world_to_px(x, y) for x, y, z in self._path_positions]

            if len(path_px) >= 2:
                draw.line(path_px, fill=(0xFF, 0x00, 0xFF), width=1)

            # Robot start position
            sx, sy = path_px[0]
            r = 5
            draw.ellipse((sx - r, sy - r, sx + r, sy + r), fill=(0x00, 0xFF, 0x00))

        png_path = self._output_dir / f"{self._team_name}_map_{ts}.png"
        img.save(str(png_path))

        # World file (.pgw)
        pgw_path = self._output_dir / f"{self._team_name}_map_{ts}.pgw"
        with open(pgw_path, "w") as f:
            f.write(f"{res}\n")
            f.write("0.0\n")
            f.write("0.0\n")
            f.write(f"{-res}\n")
            f.write(f"{ox + res * 0.5}\n")
            f.write(f"{oy + res * (h - 0.5)}\n")

        return str(png_path)

    def _export_csv_stub(self, ts: str) -> str:
        csv_path = self._output_dir / f"{self._team_name}_mission_{ts}.csv"
        with open(csv_path, "w") as f:
            f.write("pois\n")
            f.write("1.3\n")
            f.write(f"{self._team_name}\n")
            f.write(f"{self._country}\n")
            f.write(f"{time.strftime('%Y-%m-%d')}\n")
            f.write(f"{time.strftime('%H:%M:%S')}\n")
            f.write("Prelim1\n")
            f.write("detection,time,type,name,x,y,z,robot,mode\n")
        return str(csv_path)


def _unpack_pointcloud2(msg: PointCloud2) -> list[tuple[float, float, float]]:
    # Find x, y, z field offsets
    field_offsets: dict[str, int] = {}
    for field in msg.fields:
        field_offsets[field.name] = field.offset

    if "x" not in field_offsets or "y" not in field_offsets or "z" not in field_offsets:
        return []

    ox = field_offsets["x"]
    oy = field_offsets["y"]
    oz = field_offsets["z"]
    point_step = msg.point_step
    data = bytes(msg.data)
    n = msg.width * msg.height
    points = []
    for i in range(n):
        base = i * point_step
        x = struct.unpack_from("f", data, base + ox)[0]
        y = struct.unpack_from("f", data, base + oy)[0]
        z = struct.unpack_from("f", data, base + oz)[0]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    return points


def main(argv=None):
    rclpy.init(args=argv)
    node = RoboMapExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
