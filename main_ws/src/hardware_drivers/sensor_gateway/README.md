# sensor_gateway

High-speed serial bridge for STM32-based sensors, primarily for the BNO055 IMU.

Implemented in **Rust** using `rclrs` for high reliability and memory safety.

## Features

- **Serial Parsing**: Reads line-oriented CSV data from STM32 UART.
- **IMU Orientation**: Converts Euler angles (from BNO055) into quaternions (`sensor_msgs/Imu`).
- **Health Monitoring**: Publishes a heart-beat topic (`/imu/health`) based on the data reception rate.
- **Robust Serial Handling**: Uses `stty` for low-level configuration and handles device re-openings.
- **Calibration Status**: Periodically logs IMU calibration levels (Sys/Gyr/Acc/Mag).

## Topics

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Filtered orientation and raw sensor data |
| `/imu/health` | `std_msgs/Bool` | `true` if data is received within the last 2 seconds |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/stm32` | Device path |
| `baud_rate` | int | `115200` | Serial speed |

## Protocol Format

The STM32 is expected to send lines in the following CSV format:
`heading,roll,pitch,sys_cal,gyr_cal,acc_cal,mag_cal`
Values are floating-point for angles and integers (0-3) for calibration.
