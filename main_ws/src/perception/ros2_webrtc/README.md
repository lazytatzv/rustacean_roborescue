ros2_webrtc
===========

This package provides ROS2-managed WebRTC audio components: a signaling server
node and GStreamer-based robot/operator nodes that integrate with ROS2 topics
for signaling exchange.

Nodes
- `signaling_node`: runs a WebSocket signaling server on port 8080 and bridges
  incoming messages to `webrtc/incoming` topic and forwards messages published
  to `webrtc/outgoing` to connected websocket peers.
- `robot_webrtc_node`: robot-side GStreamer pipeline that publishes ICE/SDP to
  `webrtc/outgoing` and listens on `webrtc/incoming` to set remote descriptions.
- `operator_webrtc_node`: operator-side receiver, similar publish/subscribe.

Run (development)

Start ROS2 (source your workspace), then run:

```bash
# in one terminal
ros2 run ros2_webrtc signaling_node

# robot host
ros2 run ros2_webrtc robot_webrtc_node

# operator host
ros2 run ros2_webrtc operator_webrtc_node
```

Notes
- These nodes are prototypes: they require `gstreamer` and `python3-gi` to be
  installed, and ROS2 environment active. For production, add STUN/TURN, WSS,
  token auth, robustness and reconnection handling.
