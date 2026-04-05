# joy_controller

High-level teleoperation logic for converting PS4 controller inputs into robot commands.

Implemented in **C++** using `rclcpp`.

## Operation Modes

| Mode | Trigger | Description |
|------|---------|-------------|
| **STOP** | **PS Button** | **Emergency Stop.** Latch-style lockout. Publishes `/emergency_stop`. |
| **DRIVE** | **OPTIONS** | Controls crawlers and flippers. |
| **ARM (IK)** | **SHARE** (1st) | Controls the robotic arm using Cartesian velocity (IK). |
| **ARM (Direct)** | **SHARE** (2nd) | Controls the robotic arm joints directly. |

### DRIVE Mode Details
- **Left Stick Y**: Left crawler velocity.
- **Right Stick Y**: Right crawler velocity.
- **D-Pad Up/Down**: Front flippers.
- **L1/R1 / L2/R2**: Individual flipper control.

### ARM (IK) Mode Details
- **Left Stick**: Horizontal (XY) translation.
- **Right Stick Y**: Vertical (Z) translation.
- **Right Stick X**: Yaw rotation.
- **L2/R2 Triggers**: Roll rotation.
- **D-Pad Up/Down**: Pitch rotation.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Raw controller axes and buttons |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/crawler_driver` | `custom_interfaces/CrawlerVelocity` | Target m/s for crawlers |
| `/flipper_driver` | `custom_interfaces/FlipperVelocity` | Raw units for flippers |
| `/arm_cmd_vel` | `geometry_msgs/Twist` | Cartesian velocity for IK |
| `/arm_joint_cmd_vel` | `sensor_msgs/JointState` | Joint velocities for direct control |
| `/emergency_stop` | `std_msgs/Bool` | Hard stop trigger (Reliable/TransientLocal) |

## Safety Features
- **Rising-edge detection**: Buttons only trigger once per press.
- **Latched E-Stop**: The PS button locks the robot in STOP mode. Clearing require switching to another mode.
- **Deadzone handling**: Prevents ghost movements from analog sticks.
