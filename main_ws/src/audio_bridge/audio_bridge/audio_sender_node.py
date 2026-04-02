#!/usr/bin/env python3
"""
audio_sender: マイク入力を Opus にエンコードして ROS 2 トピックに publish する。

GStreamer パイプライン:
  pulsesrc → audioconvert → audioresample → opusenc → appsink → ROS 2 topic

パラメータ:
  topic       (str)  publish 先トピック名  (例: /robot/audio)
  device      (str)  PulseAudio デバイス名  (空 = システムデフォルト)
  bitrate     (int)  Opus ビットレート [bps] (デフォルト: 32000)
"""

import threading

import gi
import rclpy
from custom_interfaces.msg import AudioChunk
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst
from rclpy.node import Node
from std_msgs.msg import Header

Gst.init(None)

# Opus は 48 kHz / mono をデフォルトとする (PulseAudio のネイティブレートと一致)
_SAMPLE_RATE = 48000
_CHANNELS = 1


class AudioSenderNode(Node):
    def __init__(self) -> None:
        super().__init__("audio_sender")

        self.declare_parameter("topic", "/robot/audio")
        self.declare_parameter("device", "")
        self.declare_parameter("bitrate", 32000)

        topic = self.get_parameter("topic").value
        device: str = self.get_parameter("device").value
        bitrate: int = self.get_parameter("bitrate").value

        self._pub = self.create_publisher(AudioChunk, topic, 10)

        # --- GStreamer パイプライン構築 ---
        device_prop = f'device="{device}"' if device else ""
        pipeline_str = (
            f"pulsesrc {device_prop} "
            f"! audioconvert "
            f"! audioresample "
            f"! audio/x-raw,rate={_SAMPLE_RATE},channels={_CHANNELS},format=S16LE "
            f"! opusenc bitrate={bitrate} frame-size=20 "
            f"! appsink name=sink emit-signals=true sync=false drop=false"
        )
        self.get_logger().info(f"sender pipeline: {pipeline_str}")

        self._pipeline = Gst.parse_launch(pipeline_str)
        sink = self._pipeline.get_by_name("sink")
        sink.connect("new-sample", self._on_new_sample)

        # バスエラーを拾う
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)

        self._pipeline.set_state(Gst.State.PLAYING)

        # GLib ループをバックグラウンドスレッドで回す
        self._loop = GLib.MainLoop()
        threading.Thread(target=self._loop.run, daemon=True).start()

        self.get_logger().info(f"audio_sender ready → {topic}")

    # ------------------------------------------------------------------
    def _on_new_sample(self, appsink) -> Gst.FlowReturn:
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        data = bytes(map_info.data)
        buf.unmap(map_info)

        msg = AudioChunk()
        msg.header = Header(stamp=self.get_clock().now().to_msg())
        msg.sample_rate = _SAMPLE_RATE
        msg.channels = _CHANNELS
        msg.data = list(data)
        self._pub.publish(msg)

        return Gst.FlowReturn.OK

    def _on_bus_error(self, _bus, message) -> None:
        err, debug = message.parse_error()
        self.get_logger().error(f"GStreamer error: {err} ({debug})")

    def destroy_node(self) -> None:
        self._pipeline.set_state(Gst.State.NULL)
        self._loop.quit()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AudioSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
