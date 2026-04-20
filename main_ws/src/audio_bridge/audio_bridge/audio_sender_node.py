#!/usr/bin/env python3
"""
audio_sender: マイク入力を Opus にエンコードして ROS 2 トピックに publish する。
ゲインを極小(0.1)に設定し、ハードウェア由来のバリバリ音（割れ）を回避。
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

# Opus の推奨レートに戻す
_SAMPLE_RATE = 48000
_CHANNELS = 1

class AudioSenderNode(Node):
    def __init__(self) -> None:
        super().__init__("audio_sender")

        self.declare_parameter("topic", "/robot/audio")
        self.declare_parameter("device", "") 
        self.declare_parameter("bitrate", 32000)
        self.declare_parameter("input_gain", 0.1) # 0.1まで下げて「割れ」を確実に防ぐ

        topic = self.get_parameter("topic").value
        device = self.get_parameter("device").value
        bitrate = self.get_parameter("bitrate").value
        gain = self.get_parameter("input_gain").value

        self._pub = self.create_publisher(AudioChunk, topic, 10)
        self._sent_count = 0

        # パイプライン構成:
        # pulsesrc -> ゲイン調整(0.1) -> Opus
        device_prop = f'device="{device}"' if device else ""
        
        # 不要な処理をすべて削り、マイクの生の音を極小音量で送ります
        pipeline_str = (
            f"pulsesrc {device_prop} ! audioconvert ! audioresample "
            f"! audio/x-raw,rate={_SAMPLE_RATE},channels={_CHANNELS},format=S16LE "
            f"! volume volume={gain} "
            f"! opusenc bitrate={bitrate} frame-size=20 "
            f"! appsink name=sink emit-signals=true sync=false drop=false"
        )
        
        self.get_logger().info(f"🎙️ Audio Sender: Low-Gain Mode. Gain: {gain}")
        
        try:
            self._pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to start: {e}")
            return

        sink = self._pipeline.get_by_name("sink")
        sink.connect("new-sample", self._on_new_sample)
        
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)

        self._pipeline.set_state(Gst.State.PLAYING)
        self._loop = GLib.MainLoop()
        threading.Thread(target=self._loop.run, daemon=True).start()

    def _on_new_sample(self, appsink) -> Gst.FlowReturn:
        sample = appsink.emit("pull-sample")
        if sample is None: return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if ok:
            msg = AudioChunk()
            msg.header = Header(stamp=self.get_clock().now().to_msg())
            msg.sample_rate = _SAMPLE_RATE
            msg.channels = _CHANNELS
            msg.data = list(map_info.data)
            self._pub.publish(msg)
            buf.unmap(map_info)
            self._sent_count += 1
            if self._sent_count % 100 == 0:
                self.get_logger().info(f"📡 Sending (Gain: {self.get_parameter('input_gain').value})", once=False)
        return Gst.FlowReturn.OK

    def _on_bus_error(self, _bus, message) -> None:
        err, debug = message.parse_error()
        self.get_logger().error(f"GStreamer error: {err}")

    def destroy_node(self) -> None:
        if self._pipeline: self._pipeline.set_state(Gst.State.NULL)
        super().destroy_node()

def main() -> None:
    rclpy.init()
    node = AudioSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
