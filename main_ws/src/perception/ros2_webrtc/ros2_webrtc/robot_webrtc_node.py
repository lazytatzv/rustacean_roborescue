#!/usr/bin/env python3
"""
Robot ROS2 node: GStreamer WebRTC sender/receiver integrated with ROS2
signaling topics. Publishes local SDP/ICE to `webrtc/outgoing` and subscribes
to `webrtc/incoming` for remote data. Hardened with CLI args, backoff and
improved logging.
"""

import argparse
import json
import os
import queue
import sys
import time

import rclpy
from gi.repository import GLib, Gst
from rclpy.node import Node
from std_msgs.msg import String

Gst.init(None)

IN_Q = queue.Queue()


class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_webrtc")
        self.pub = self.create_publisher(String, "webrtc/outgoing", 10)
        self.sub = self.create_subscription(
            String, "webrtc/incoming", self.on_incoming, 10
        )

    def on_incoming(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        IN_Q.put(data)


def create_pipeline(audio_device=None, bitrate=64000):
    pipeline = Gst.Pipeline.new("robot-webrtc")
    source = Gst.ElementFactory.make("pulsesrc", "source")
    if audio_device:
        source.set_property("device", audio_device)
    conv = Gst.ElementFactory.make("audioconvert", "conv")
    res = Gst.ElementFactory.make("audioresample", "res")
    enc = Gst.ElementFactory.make("opusenc", "enc")
    enc.set_property("bitrate", int(bitrate))
    pay = Gst.ElementFactory.make("rtpopuspay", "pay")
    queue_e = Gst.ElementFactory.make("queue", "queue")
    webrtc = Gst.ElementFactory.make("webrtcbin", "webrtc")
    if not all([source, conv, res, enc, pay, queue_e, webrtc]):
        print("Missing GStreamer elements", file=sys.stderr)
        sys.exit(1)

    pipeline.add(source)
    pipeline.add(conv)
    pipeline.add(res)
    pipeline.add(enc)
    pipeline.add(pay)
    pipeline.add(queue_e)
    pipeline.add(webrtc)

    source.link(conv)
    conv.link(res)
    res.link(enc)
    enc.link(pay)
    pay.link(queue_e)
    queue_e.link(webrtc)

    return pipeline, webrtc


def publish_sdp(node, sdp_text, typ="sdp"):
    node.get_logger().info("Publishing SDP")
    node.pub.publish(
        String(
            data=json.dumps(
                {
                    "type": typ,
                    "sdp": sdp_text,
                    "room": os.environ.get("ROBOT_ROOM", "robot"),
                }
            )
        )
    )


def webrtc_on_ice(webrtc, mlineindex, candidate, node):
    node.get_logger().debug("ICE candidate gathered")
    node.pub.publish(
        String(
            data=json.dumps(
                {
                    "type": "ice",
                    "candidate": candidate,
                    "sdpMLineIndex": mlineindex,
                    "room": os.environ.get("ROBOT_ROOM", "robot"),
                }
            )
        )
    )


def handle_incoming(webrtc, node):
    try:
        while True:
            data = IN_Q.get_nowait()
            t = data.get("type")
            if t == "sdp":
                # Assume incoming is answer
                sdp = data.get("sdp")
                try:
                    s = Gst.SDP.SDPMessage.new()
                    Gst.SDP.sdp_message_parse_buffer(sdp.encode("utf-8"), s)
                    desc = Gst.WebRTCSessionDescription.new(Gst.WebRTCSDPType.ANSWER, s)
                    webrtc.emit("set-remote-description", desc, None)
                    node.get_logger().info("Set remote description (answer)")
                except Exception as e:
                    node.get_logger().error(f"Error setting remote SDP: {e}")
            elif t == "ice":
                try:
                    webrtc.emit(
                        "add-ice-candidate",
                        data.get("sdpMLineIndex"),
                        data.get("candidate"),
                    )
                except Exception:
                    pass
    except queue.Empty:
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--audio-device", default=None)
    parser.add_argument("--bitrate", default=64000)
    args = parser.parse_args()

    rclpy.init()
    node = RobotNode()

    backoff = 1
    while rclpy.ok():
        try:
            pipeline, webrtc = create_pipeline(
                audio_device=args.audio_device, bitrate=args.bitrate
            )
            webrtc.connect(
                "on-ice-candidate", lambda w, m, c: webrtc_on_ice(w, m, c, node)
            )

            # create offer and publish
            def on_offer_created(promise, webrtc):
                reply = promise.get_reply()
                offer = reply.get_value("offer")
                webrtc.emit("set-local-description", offer, None)
                sdp = offer.sdp.as_text()
                publish_sdp(node, sdp, typ="sdp")

            webrtc.connect("notify::ice-gathering-state", lambda *a: None)
            promise = Gst.Promise.new_with_change_func(on_offer_created, webrtc)
            webrtc.emit("create-offer", None, promise)

            pipeline.set_state(Gst.State.PLAYING)

            backoff = 1
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                handle_incoming(webrtc, node)
                GLib.MainContext.default().iteration(False)
        except Exception as e:
            node.get_logger().error(f"Pipeline error: {e}, restarting in {backoff}s")
            try:
                pipeline.set_state(Gst.State.NULL)
            except Exception:
                pass
            time.sleep(backoff)
            backoff = min(backoff * 2, 30)

    try:
        pipeline.set_state(Gst.State.NULL)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
