# audio_bridge

`audio_bridge` is the workspace package for bidirectional audio transport over ROS 2 topics. It keeps voice traffic inside the same ROS/Zenoh transport path as the rest of the robot stack, so there is no separate WebRTC signaling layer.

## What It Does

- `audio_sender` captures microphone input, encodes it with Opus, and publishes `custom_interfaces/msg/AudioChunk` messages.
- `audio_receiver` subscribes to the operator audio topic, decodes Opus, and plays the stream to the robot speakers.
- The default topic pair is `/robot/audio` and `/operator/audio`.

## Launch

The normal entry point is [bringup/launch/audio.launch.py](../bringup/launch/audio.launch.py).

```bash
cd main_ws
source install/setup.bash
ros2 launch bringup audio.launch.py
```

Useful launch arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `use_audio` | `true` | Enable or disable the audio bridge nodes |
| `robot_mic_device` | empty | PulseAudio capture device name on the robot side |
| `robot_spk_device` | empty | PulseAudio playback device name on the robot side |
| `bitrate` | `32000` | Opus bitrate in bps |

## Direct Execution

```bash
ros2 run audio_bridge audio_sender --ros-args -p topic:=/robot/audio
ros2 run audio_bridge audio_receiver --ros-args -p topic:=/operator/audio
```

## Implementation Notes

- The sender path is `PulseAudio -> GStreamer -> Opus -> ROS 2`.
- The receiver path is `ROS 2 -> GStreamer -> Opus decode -> PulseAudio`.
- Both nodes use a small GLib main loop in a background thread so the GStreamer bus keeps running while rclpy spins.
- The package depends on `custom_interfaces/msg/AudioChunk` and is designed to be used together with the Zenoh-based operator bridge.
