# crawler_driver

High-performance driver for Roboclaw motor controllers, designed for heavy-duty crawler tracks.

Implemented in **C++** using `rclcpp` and `boost::asio`.

## Features & Logic

### 1. PID Velocity Control
The driver bypasses raw PWM and uses Roboclaw's internal PID velocity loop. It maps ROS 2 velocity commands (m/s) to Quadrature Pulses Per Second (QPPS) based on the crawler's physical dimensions:
- `counts_per_meter = (counts_per_rev * gear_ratio * pulley_ratio) / circumference`

### 2. Stall Protection Algorithm
To prevent motor burnout during collisions or when stuck in rubble, the driver implements a software-level stall detection:
- **Error Ratio**: If `abs(commanded - actual) > commanded * stall_error_ratio`, a stall is suspected.
- **Score Accumulation**: A "stall score" increases while the error persists.
- **Output Scaling**: As the score increases, the driver automatically scales down the output power (up to `stall_max_reduction`, e.g., 50%) to protect the motors.
- **Recovery**: Once the error disappears, the output scale returns to 1.0 at double the speed of accumulation.

### 3. Hardware Safety & Watchdog
- **Command Watchdog**: If no valid ROS 2 message is received for $> 500ms$ (default), the driver sends a hard-stop command to the Roboclaw.
- **Hardware Timeout**: The Roboclaw itself is configured with a serial timeout. If the NUC crashes or the USB cable is unplugged, the controller will stop the motors autonomously.
- **Thermal & Voltage Monitoring**: Telemetry is polled at 10Hz. Warnings are logged if the battery voltage is low or the driver temperature exceeds safety limits.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/crawler_driver` | `CrawlerVelocity` | Direct left/right crawler velocity [m/s]. |
| `/cmd_vel` | `geometry_msgs/Twist` | Standard ROS 2 drive command (Diff-drive math applied). |
| `/emergency_stop` | `std_msgs/Bool` | Hard stop trigger (Reliable/TransientLocal). |

### Publications (Diagnostics)
| Topic | Type | Description |
|-------|------|-------------|
| `~/diagnostics/m1_current_ma` | `Float64` | Motor 1 current (Real-time). |
| `~/diagnostics/m1_speed_actual`| `Float64` | Measured speed in QPPS. |
| `~/diagnostics/m1_stall_scale` | `Float64` | Current power scaling (1.0 = normal, 0.5 = stalled). |
| `~/diagnostics/battery_volts` | `Float64` | Main 12-24V battery voltage. |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/roboclaw` | Path to the USB-Serial device. |
| `baud_rate` | `115200` | Baud rate (must match Roboclaw firmware). |
| `m1_kp / ki / kd` | - | Velocity PID gains for Motor 1 (Left). |
| `m2_kp / ki / kd` | - | Velocity PID gains for Motor 2 (Right). |
| `stall_error_ratio` | `0.5` | Threshold for speed error to be considered a stall. |
| `stall_max_reduction` | `0.5` | Max power drop during stall events. |
