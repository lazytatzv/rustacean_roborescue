# arm_controller

Professional-grade velocity-resolved Inverse Kinematics (IK) and Joint Control for 6-DOF robotic arms.

Implemented in **Rust** using `rclrs` and the `k` kinematics library.

## Features

- **Jacobian-based Velocity IK**: Uses Damped Least Squares (DLS) for robust control near singularities.
- **Adaptive Damping**: Automatically increases damping ($\lambda$) based on manipulability measure.
- **Joint Limit Avoidance**: Direction-aware velocity scaling near hard limits + Null-space repulsion towards neutral posture.
- **Integrator Control**: Internally integrates velocity commands to produce smooth target positions, while "tethering" to actual feedback to prevent drift.
- **Safety**:
  - **Watchdog**: Stops movement if command is lost for > 500ms.
  - **E-Stop**: Instant lockout upon receiving `/emergency_stop`.
  - **Feedback Interlock**: Only accepts commands once full joint feedback is received.
- **Diagnostics**: Publishes loop timing, manipulability, and joint feedback rates.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/arm_cmd_vel` | `geometry_msgs/Twist` | Target end-effector velocity (IK Mode) |
| `/arm_joint_cmd_vel` | `sensor_msgs/JointState` | Target joint velocities (Direct Mode) |
| `/joint_states` | `sensor_msgs/JointState` | Actual joint positions (Feedback) |
| `/emergency_stop` | `std_msgs/Bool` | Hard stop trigger (Reliable/TransientLocal) |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Computed target positions and velocities |
| `/arm_ee_pose` | `geometry_msgs/PoseStamped` | Forward Kinematics result (End-effector pose) |
| `/arm_controller/diagnostics/*` | `std_msgs/Float64` | Loop time, manipulability, and JS rate |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `urdf_path` | string | (Required) | Path to the robot's URDF file |
| `end_link` | string | `link_tip` | Name of the end-effector link |
| `control_period_ms` | int | `20` | Control loop period (50Hz) |
| `dls_lambda_base` | double | `0.01` | Base damping constant |
| `joint_vel_limit` | double | `1.5` | Max joint velocity [rad/s] |
| `manipulability_threshold` | double | `0.005` | Threshold to start increasing damping |

## Build & Run

Ensure you are in the `main_ws` and the ROS 2 environment is sourced.

```bash
# Build
just forge-packages arm_controller

# Run (stand-alone)
ros2 run arm_controller arm_controller --ros-args -p urdf_path:=$(pwd)/src/bringup/urdf/sekirei.urdf
```
