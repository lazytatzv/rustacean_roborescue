#!/usr/bin/env python3
"""
audio_receiver: ROS 2 トピックから Opus パケットを受信してスピーカーに出力する。

GStreamer パイプライン:
  ROS 2 topic → appsrc → opusdec → audioconvert → audioresample → pulsesink

パラメータ:
  topic   (str)  subscribe するトピック名  (例: /operator/audio)
  device  (str)  PulseAudio デバイス名    (空 = システムデフォルト)
"""

import threading

import rclpy
from custom_interfaces.msg import AudioChunk
from gi.repository import GLib, Gst
from rclpy.node import Node

Gst.init(None)

# sender 側と合わせた固定値
_OPUS_CAPS = (
    "audio/x-opus,"
    "rate=(int)48000,"
    "channels=(int)1,"
    "channel-mapping-family=(int)0"
)


class AudioReceiverNode(Node):
    def __init__(self) -> None:
        super().__init__("audio_receiver")

        self.declare_parameter("topic", "/operator/audio")
        self.declare_parameter("device", "")

        topic = self.get_parameter("topic").value
        device: str = self.get_parameter("device").value

        self._sub = self.create_subscription(AudioChunk, topic, self._on_audio, 10)

        # --- GStreamer パイプライン構築 ---
        device_prop = f'device="{device}"' if device else ""
        pipeline_str = (
            f"appsrc name=src is-live=true format=time "
            f"caps={_OPUS_CAPS} "
            f"! queue max-size-buffers=10 leaky=downstream "
            f"! opusdec "
            f"! audioconvert "
            f"! audioresample "
            f"! pulsesink {device_prop} sync=false"
        )
        self.get_logger().info(f"receiver pipeline: {pipeline_str}")

        self._pipeline = Gst.parse_launch(pipeline_str)
        self._appsrc: Gst.Element = self._pipeline.get_by_name("src")

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)

        self._pipeline.set_state(Gst.State.PLAYING)

        self._loop = GLib.MainLoop()
        threading.Thread(target=self._loop.run, daemon=True).start()

        self.get_logger().info(f"audio_receiver ready ← {topic}")

    # ------------------------------------------------------------------
    def _on_audio(self, msg: AudioChunk) -> None:
        data = bytes(msg.data)
        buf = Gst.Buffer.new_wrapped(data)

        # sender のタイムスタンプを PTS として引き継ぐ
        stamp = msg.header.stamp
        buf.pts = stamp.sec * Gst.SECOND + stamp.nanosec

        ret = self._appsrc.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().warning(f"appsrc push-buffer returned {ret}")

    def _on_bus_error(self, _bus, message) -> None:
        err, debug = message.parse_error()
        self.get_logger().error(f"GStreamer error: {err} ({debug})")

    def destroy_node(self) -> None:
        self._appsrc.emit("end-of-stream")
        self._pipeline.set_state(Gst.State.NULL)
        self._loop.quit()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AudioReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
