# arm_controller

Professional-grade velocity-resolved Inverse Kinematics (IK) and Joint Control for 6-DOF robotic arms.

Implemented in **Rust** using `rclrs` and the `k` kinematics library.

## Features & Theory of Operation

### 1. Jacobian-based Velocity IK
This node converts end-effector velocity commands ($\dot{x}$) into joint velocities ($\dot{q}$) using the Jacobian matrix ($J$).
- **Damped Least Squares (DLS)**: To handle singularities, we use the formula: $\dot{q} = J^T (JJ^T + \lambda^2 I)^{-1} \dot{x}$
- **Adaptive Damping**: The damping factor ($\lambda$) is dynamically adjusted based on the manipulability measure ($w = \sqrt{\det(JJ^T)}$). When $w$ drops below a threshold, $\lambda$ increases to prioritize stability over tracking precision.

### 2. Joint Limit & Collision Avoidance
- **Direction-aware Scaling**: As a joint approaches its soft limit, its allowable velocity is scaled down to zero, but only in the direction of the limit.
- **Null-space Projection**: We exploit the redundant DOF (if any) to push the arm towards a "neutral" posture using the null-space of the Jacobian: $\dot{q}_{total} = \dot{q}_{task} + (I - J^\dagger J) \dot{q}_{repulsion}$.

### 3. High-Performance Architecture
- **Non-blocking Worker**: To ensure the ROS 2 timer callback remains deterministic (50Hz), the heavy SVD/Jacobian math is delegated to a background worker thread via a single-slot lock-free channel.
- **Tethered Integration**: Command integration happens on the target positions, but the target is "tethered" to the actual joint feedback. If the target drifts too far (e.g., due to a collision), it is snapped back to prevent integral windup.

## Topics

### Subscriptions
| Topic | Type | Description |
|-------|------|-------------|
| `/arm_cmd_vel` | `geometry_msgs/Twist` | Target end-effector velocity (IK Mode). |
| `/arm_joint_cmd_vel` | `sensor_msgs/JointState` | Target joint velocities (Direct Mode). |
| `/joint_states` | `sensor_msgs/JointState` | Actual joint positions. **Required for the node to "arm".** |
| `/emergency_stop` | `std_msgs/Bool` | Reliable/TransientLocal latch. Disables all outputs. |

### Publications
| Topic | Type | Description |
|-------|------|-------------|
| `/arm_joint_commands` | `sensor_msgs/JointState` | Computed target positions + velocities. |
| `/arm_ee_pose` | `geometry_msgs/PoseStamped` | Real-time Forward Kinematics (FK) result. |
| `/arm_controller/diagnostics/manipulability` | `std_msgs/Float64` | Current arm manipulability (Low = near singularity). |
| `/arm_controller/diagnostics/loop_time_ms` | `std_msgs/Float64` | Control loop latency. |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `urdf_path` | (Required) | Absolute path to the URDF file for kinematics. |
| `end_link` | `link_tip` | The link name to be controlled. |
| `control_period_ms` | `20` | Loop period. 50Hz is recommended for Dynamixel. |
| `joint_vel_limit` | `1.5` | Absolute cap on joint speed [rad/s]. |
| `joint_limit_margin` | `0.1` | Buffer zone [rad] before hard limits where scaling kicks in. |
| `null_space_repulsion_gain` | `0.5` | Strength of the push towards neutral posture. |
