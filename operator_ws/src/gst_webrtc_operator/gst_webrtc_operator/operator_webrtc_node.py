#!/usr/bin/env python3
"""
Operator ROS2 node: receives WebRTC offer from robot via `webrtc/outgoing`,
sets remote, creates answer and publishes SDP/ICE on `webrtc/incoming`.
Also handles incoming ICE candidates and plays audio to the local sink.
"""

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


class OperatorNode(Node):
    def __init__(self):
        super().__init__("operator_webrtc")
        self.pub = self.create_publisher(String, "webrtc/incoming", 10)
        self.sub = self.create_subscription(
            String, "webrtc/outgoing", self.on_outgoing, 10
        )

    def on_outgoing(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        IN_Q.put(data)


def publish_sdp(node, sdp_text, typ="sdp"):
    node.get_logger().info("Publishing SDP answer")
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
    node.get_logger().debug("ICE candidate gathered (operator)")
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
                sdp = data.get("sdp")
                try:
                    s = Gst.SDP.SDPMessage.new()
                    Gst.SDP.sdp_message_parse_buffer(sdp.encode("utf-8"), s)
                    desc = Gst.WebRTCSessionDescription.new(Gst.WebRTCSDPType.OFFER, s)
                    webrtc.emit("set-remote-description", desc, None)
                    node.get_logger().info("Set remote description (offer)")

                    # create answer
                    def on_answer_created(promise, webrtc):
                        reply = promise.get_reply()
                        answer = reply.get_value("answer")
                        webrtc.emit("set-local-description", answer, None)
                        sdp_text = answer.sdp.as_text()
                        publish_sdp(node, sdp_text, typ="sdp")

                    promise = Gst.Promise.new_with_change_func(
                        on_answer_created, webrtc
                    )
                    webrtc.emit("create-answer", None, promise)
                except Exception as e:
                    node.get_logger().error(f"Error handling offer SDP: {e}")
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


def on_incoming_stream(webrtc, pad, pipeline, node):
    if pad.get_direction() != Gst.PadDirection.SRC:
        return
    node.get_logger().info("Incoming stream pad-added: linking to audio sink")
    queue = Gst.ElementFactory.make("queue", None)
    decode = Gst.ElementFactory.make("decodebin", None)
    audioconv = Gst.ElementFactory.make("audioconvert", None)
    audiores = Gst.ElementFactory.make("audioresample", None)
    sink = Gst.ElementFactory.make("autoaudiosink", None)

    for e in (queue, decode, audioconv, audiores, sink):
        pipeline.add(e)
        e.sync_state_with_parent()

    sink_pad = queue.get_static_pad("sink")
    pad.link(sink_pad)
    queue.link(decode)

    def on_decode_pad(db, dbpad):
        try:
            dbpad.link(audioconv.get_static_pad("sink"))
            audioconv.link(audiores)
            audiores.link(sink)
        except Exception:
            pass

    decode.connect("pad-added", on_decode_pad)


def create_pipeline():
    pipeline = Gst.Pipeline.new("operator-webrtc")
    webrtc = Gst.ElementFactory.make("webrtcbin", "webrtc")
    if not webrtc:
        print("Missing webrtcbin element", file=sys.stderr)
        sys.exit(1)
    pipeline.add(webrtc)
    return pipeline, webrtc


def main():
    rclpy.init()
    node = OperatorNode()

    backoff = 1
    while rclpy.ok():
        try:
            pipeline, webrtc = create_pipeline()
            webrtc.connect(
                "on-ice-candidate", lambda w, m, c: webrtc_on_ice(w, m, c, node)
            )
            webrtc.connect(
                "pad-added", lambda w, p: on_incoming_stream(w, p, pipeline, node)
            )

            pipeline.set_state(Gst.State.PLAYING)

            backoff = 1
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                handle_incoming(webrtc, node)
                GLib.MainContext.default().iteration(False)
        except Exception as e:
            node.get_logger().error(
                f"Operator pipeline error: {e}, restarting in {backoff}s"
            )
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
