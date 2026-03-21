Detailed internal documentation — arm_driver

1. Purpose and scope

This document describes internal architecture, control flow, data structures, and extension points for the `arm_driver` crate which provides a Dynamixel-based arm + gripper driver.

2. High-level flow

- `main.rs::run()` initializes an `rclrs::Context`, creates a basic executor, and creates the node `arm_gripper_driver`.
- Parameters are declared and read using the `node.declare_parameter(...).default(...).mandatory()?.get()` pattern. The code uses rclrs-compatible defaults (`Arc<str>`, `Arc<[...]>`) internally and converts to standard Rust types.
- Publishers and subscriptions are created for joint states, gripper status, arm joint commands, and gripper commands.
- A hardware thread is spawned to handle direct motor I/O, passing the required publishers and a receiver for `HwCommand`.
- The executor spins using `executor.spin(SpinOptions::default())` to process ROS callbacks.

3. Key types

- `HwCommand` — container passed across threads containing optional `arm_positions: Option<Vec<f64>>` and `gripper_position: Option<f64>`.
- `ArmDynamixelDriver` — abstraction around motor-specific serial protocol; implements:
  - `new(port: &str, baud: u32, arm_ids: Vec<u8>, gripper_id: u8)`
  - `init_motors(profile_velocity: u32, gripper_max_current: u16)`
  - `read_arm_positions() -> Result<Vec<f64>>`
  - `write_arm_positions(&[f64])`, `write_gripper_position(f64)`
  - `read_gripper_status()`
  - `emergency_stop()`
  - `torque_off_all()`

4. Hardware thread behavior

- Runs at `HW_LOOP_HZ` (50Hz). Each loop:
  - Checks `estop_flag`: if set, calls `emergency_stop()` and waits for shutdown.
  - Drains command channel (`rx_cmd.try_recv()`) and applies any arm/gripper writes.
  - Reads arm positions and publishes a `JointState` with `header.stamp` set by `now_stamp()`.
  - Reads gripper status and publishes `GripperStatus` message.
  - Sleeps to maintain loop rate.

5. Parameter handling details

- For string and array parameter defaults we use `Arc<str>` and `Arc<[Arc<str>]>` to satisfy `rclrs::ParameterVariant` bounds. After retrieving, the code converts these to `String` and `Vec<String>` for convenience.
- `arm_ids` are read as `Arc<[i64]>` then mapped to `Vec<u8>`; validate IDs are in the `0..=255` range when using nonstandard IDs.

6. Extending

- To add velocity control, extend `HwCommand` with `arm_velocities: Option<Vec<f64>>` and propagate through subscriptions.
- To support additional motor models, add feature flags to `driver` and factor device-specific logic behind a trait.

7. Testing

- Unit tests for `driver` may mock serial transport. Integration tests require hardware or a bus simulator.
- Suggested tests:
  - Parameter parsing: ensure various parameter shapes (single, arrays) parse into expected Rust types.
  - Command translation: subscription callback correctly maps `JointState` message into `HwCommand` positions in the expected order.

8. Common pitfalls

- Mismatch of `arm_joints` and `arm_ids` lengths — code assumes same ordering and length; consider asserting and logging at startup.
- Blocking I/O on serial port can stall the hardware thread. Keep driver I/O non-blocking or guarded with reasonable timeouts.
- If `rclrs` API changes upstream, parameter types and `spin()` signature may need adjustment.

9. TODOs

- Add runtime validation for parameter shapes and lengths with clear error messages.
- Add diagnostics topic publishing current hardware health.
- Add unit tests that mock `ArmDynamixelDriver`.
