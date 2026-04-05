# omron_bridge

Bridge for the OMRON 2JCIE-BU01 USB environmental sensor.

Implemented in **Python**.

## Features

- **Polls data**: Reads temperature, humidity, light, and pressure from the sensor.
- **POI Export**: Automatically logs "Heat Signal" detections to CSV/CSV for RoboCup Rescue competition if temperature exceeds a threshold.
- **Odom integration**: Subscribes to `/odom` to tag POI detections with robot coordinates.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Used for spatial tagging of detections |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `omron/temperature` | `std_msgs/Float32` | Temp in Celsius |
| `omron/humidity` | `std_msgs/Float32` | Relative humidity % |
| `omron/light` | `std_msgs/Float32` | Luminance in lx |
| `omron/pressure` | `std_msgs/Float32` | Pressure in hPa |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyUSB0` | Device path |
| `poll_period` | double | `1.0` | Sampling interval [s] |
| `heat_threshold_c`| double | `40.0` | Temperature to trigger Heat POI |
| `enable_poi_export`| bool | `true` | Save detections to file |
