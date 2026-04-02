# joy_controller

`joy_controller` is the joystick-to-command bridge for the robot. It converts `/joy` input into crawler, flipper, and arm commands, and it is launched from [bringup/launch/control.launch.py](../../bringup/launch/control.launch.py).

## Modes

| Button | Mode | Purpose |
|--------|------|---------|
| `PS` | `STOP` | Latches the emergency stop and zeroes all outputs |
| `OPTIONS` | `DRIVE` | Drives the crawler and flipper actuators |
| `SHARE` | `ARM` | Sends arm velocity commands to the IK pipeline |

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/joy` | `sensor_msgs/msg/Joy` |
| Publish | `/crawler_driver` | `custom_interfaces/msg/CrawlerVelocity` |
| Publish | `/flipper_driver` | `custom_interfaces/msg/FlipperVelocity` |
| Publish | `/arm_cmd_vel` | `geometry_msgs/msg/Twist` |
| Publish | `/arm_joint_cmd_vel` | `sensor_msgs/msg/JointState` |
| Publish | `/gripper_cmd` | `custom_interfaces/msg/GripperCommand` |
| Publish | `/emergency_stop` | `std_msgs/msg/Bool` |

## Quick Run

```bash
cd main_ws
source install/setup.bash
ros2 run joy_controller joy_controller_node
```

## Notes

- Button edges are handled explicitly so holding a button does not flood the downstream nodes.
- `PS` triggers E-STOP, and `OPTIONS` or `SHARE` clears it and switches mode.
- The package is tuned for the current Jazzy workspace and the launch wiring in `bringup`.

## Troubleshooting (crawler does not move)

- Verify `/joy` is arriving: `ros2 topic hz /joy`
- Verify drive command is published after pressing `OPTIONS`: `ros2 topic echo /crawler_driver --once`
- If `OPTIONS` does not switch mode, inspect button indices from `ros2 topic echo /joy` while pressing buttons.
