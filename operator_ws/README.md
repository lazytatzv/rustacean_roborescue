# Operator Station Workspace

The `operator_ws` is a lightweight ROS 2 workspace designed to run on the operator's PC. It provides the human-machine interface (HMI), controller mapping, and bridge to the robot's Zenoh network.

## Features

- **Zenoh Connectivity**: Configured as a Zenoh `peer` or `client` to interact with the robot's `router`.
- **HMI Integration**: Hosts the `foxglove_bridge` to provide data to Foxglove Studio.
- **Controller Bridge**: Maps physical PS4 controller inputs to `/joy` topics via `joy_node`.
- **Bi-directional Audio**: Uses `audio_bridge` to stream Opus-compressed audio between the operator and the robot.

## Component Overview

| Node | Purpose | Key Topics |
|------|---------|------------|
| `foxglove_bridge` | High-speed WebSocket gateway for visualization. | All robot topics → Foxglove |
| `joy_node` | Reads USB/Bluetooth gamepads. | `/joy` |
| `operator_audio_sender` | Streams mic input to the robot. | `/operator/audio` |
| `operator_audio_receiver` | Plays audio received from the robot. | `/robot/audio` |

## Setup & Operation

### 1. Networking (Crucial)
You must configure `zenoh_ope.json5` with the robot's actual IP address.
For secure QUIC communication, you also need the mTLS CA certificate from the robot.

```bash
# Get the certificate via scp
just get-cert <ROBOT_IP>
```

### 2. Launching
Use the provided Just recipes for a consistent experience:

```bash
# Start the full operator station (Joy + Bridge + Audio)
just launch

# (Optional) Start Foxglove Studio UI via Docker (runs on http://localhost:8080)
# This command should be run from the repository root
cd ..
just foxglove
cd operator_ws

# Start without audio (for low-bandwidth environments)
ros2 launch launch/operator.launch.py use_audio:=false
```

### 3. Monitoring Topics
To use standard ROS 2 CLI tools (like `ros2 topic echo`), you must set up the Zenoh environment in your terminal:

```bash
# One-time setup for the current terminal session
eval $(just set-env)

# Use Just shortcuts
just topics
just topic-info /joy
```

## Security (mTLS)

The operator station uses **Mutual TLS** for the QUIC transport. 
- The robot acts as the CA and issues its own server certificate.
- The operator must have the robot's `ca.crt` (saved as `quic/server.crt`) to verify the connection.
- This prevents unauthorized peers from subscribing to sensitive robot data or sending malicious control commands.
