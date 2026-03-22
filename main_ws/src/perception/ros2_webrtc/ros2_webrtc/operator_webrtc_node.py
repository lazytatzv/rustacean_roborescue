#!/usr/bin/env python3
"""
Operator ROS2 node: GStreamer WebRTC receiver integrated with ROS2 signaling
topics. Subscribes to `webrtc/incoming` and publishes `webrtc/outgoing`.
"""

import json
import queue
import sys

import rclpy
from gi.repository import GLib, Gst
from rclpy.node import Node
from std_msgs.msg import String

Gst.init(None)

IN_Q = queue.Queue()


class OperatorNode(Node):
    def __init__(self):
        super().__init__("operator_webrtc")
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


def create_pipeline():
    pipeline = Gst.Pipeline.new("operator-webrtc")
    webrtc = Gst.ElementFactory.make("webrtcbin", "webrtc")
    if not webrtc:
        print("webrtcbin missing", file=sys.stderr)
        sys.exit(1)
    pipeline.add(webrtc)
    return pipeline, webrtc


def webrtc_on_ice(webrtc, mlineindex, candidate, node):
    node.pub.publish(
        String(
            data=json.dumps(
                {
                    "type": "ice",
                    "candidate": candidate,
                    "sdpMLineIndex": mlineindex,
                    "room": "robot",
                }
            )
        )
    )


def on_offer_and_answer(webrtc, data, node):
    # data contains 'sdp' field (offer)
    sdp = data.get("sdp")
    s = Gst.SDP.SDPMessage.new()
    Gst.SDP.sdp_message_parse_buffer(sdp.encode("utf-8"), s)
    desc = Gst.WebRTCSessionDescription.new(Gst.WebRTCSDPType.OFFER, s)
    webrtc.emit("set-remote-description", desc, None)

    promise = Gst.Promise.new_with_change_func(lambda p, w: None, webrtc)
    webrtc.emit("create-answer", None, promise)


def handle_incoming(webrtc, node):
    try:
        while True:
            data = IN_Q.get_nowait()
            if data.get("type") == "sdp":
                on_offer_and_answer(webrtc, data, node)
            elif data.get("type") == "ice":
                webrtc.emit(
                    "add-ice-candidate",
                    data.get("sdpMLineIndex"),
                    data.get("candidate"),
                )
    except queue.Empty:
        pass


def main():
    rclpy.init()
    node = OperatorNode()

    pipeline, webrtc = create_pipeline()
    webrtc.connect("on-ice-candidate", lambda w, m, c: webrtc_on_ice(w, m, c, node))

    pipeline.set_state(Gst.State.PLAYING)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            handle_incoming(webrtc, node)
            GLib.MainContext.default().iteration(False)
    except KeyboardInterrupt:
        pass

    pipeline.set_state(Gst.State.NULL)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
