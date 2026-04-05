# crawler_driver

Low-level driver for Roboclaw motor controllers to drive the main crawler tracks.

Implemented in **C++** using `rclcpp` and `boost::asio` for serial communication.

## Features

- **PID Velocity Control**: Sends QPPS (Quadratic Pulses Per Second) commands to Roboclaw for precise speed control.
- **Differential Drive Math**: Automatically converts `/cmd_vel` (Twist) into individual motor speeds.
- **Stall Protection**: Detects motor stalls by comparing commanded vs. actual speed. Reduces output power to prevent motor burnout.
- **Hardware Telemetry**: Publishes current, voltage, temperature, and status flags from the Roboclaw.
- **Safety**:
  - **Watchdog**: Stops motors if command is lost for > 500ms.
  - **E-Stop**: Instant halt upon receiving `/emergency_stop`.
  - **Fail-safe Reconnect**: Automatically re-opens the serial port if a communication error is detected.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/crawler_driver` | `custom_interfaces/CrawlerVelocity` | Direct m/s command per crawler |
| `/cmd_vel` | `geometry_msgs/Twist` | Autonomous velocity command |
| `/emergency_stop` | `std_msgs/Bool` | Hard stop trigger |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `~/diagnostics/m1_current_ma` | `std_msgs/Float64` | Motor 1 current |
| `~/diagnostics/m1_speed_actual` | `std_msgs/Float64` | Motor 1 measured speed [qpps] |
| `~/diagnostics/temperature_c` | `std_msgs/Float64` | Controller temperature |
| `~/diagnostics/battery_volts` | `std_msgs/Float64` | Main battery voltage |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/roboclaw` | Device path |
| `baud_rate` | int | `115200` | Serial speed |
| `track_width` | double | `0.4` | Distance between tracks [m] |
| `m1_kp / ki / kd` | double | - | PID gains for Motor 1 |
| `stall_max_reduction`| double | `0.5` | Max power reduction during stall |
