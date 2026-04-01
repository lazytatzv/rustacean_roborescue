# h264_republisher

`h264_republisher` converts the H.264 camera stream published by the camera pipeline into a raw `sensor_msgs/msg/Image`. It is mainly a compatibility shim for tools that expect uncompressed images.

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/camera/image_raw/ffmpeg` | `sensor_msgs/msg/CompressedImage` |
| Publish | `/camera/image_raw` | `sensor_msgs/msg/Image` |

## Build

```bash
cd main_ws
colcon build --packages-select h264_republisher
source install/setup.bash
```

## Run

```bash
ros2 run h264_republisher h264_republisher
```

## Why It Exists

- The camera stack in `bringup` publishes compressed H.264 output for bandwidth efficiency.
- Some ROS tools and debug workflows still expect a raw `Image` topic.
- This node does the decode once and republishes the result without changing the rest of the camera pipeline.

## Dependencies

- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- FFmpeg libraries: `libavformat`, `libavcodec`, `libswscale`, `libavutil`
- OpenCV
