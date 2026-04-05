# Rustacean RoboRescue

A high-performance, multi-language ROS 2 platform for rescue robots, featuring a robust **Rust** core for low-level control and **Zenoh** for low-latency remote operation.

![System Topology](docs/images/system_topology.png)

## Core Technologies

- **Communication**: Zenoh (`rmw_zenoh_cpp`) with QUIC mTLS for secure, low-latency telemetry over unstable Wi-Fi.
- **Hardware Drivers**: Safety-critical drivers (IMU, Arm) implemented in **Rust**; high-speed motion control (Tracks, Flippers) in **C++**.
- **Perception**: LiDAR-IMU fusion (FAST-LIO2), SLAM (slam_toolbox), and hardware-accelerated QR detection.
- **Operation**: Foxglove Studio integration + dual-mode (IK/Direct) arm control with PS4 controller.
- **Environment**: Nix-based reproducible development environment.

## Repository Structure

- `main_ws/`: Robot-side ROS 2 workspace (Drivers, Control, SLAM).
- `operator_ws/`: Operator Station workspace (UI, Controller Bridge).
- `docs/`: Technical documentation and operation manuals.
- `deploy/`: Deployment scripts for Intel NUC and udev rules.
- `stm32_ws/`: Embedded firmware for sensor processing.

## Quick Setup

### 1. Requirements
- Linux (Ubuntu 24.04 or NixOS recommended)
- [Nix](https://nixos.org/download.html) with Flakes enabled.

### 2. Enter Development Environment
```bash
nix develop --accept-flake-config
```

### 3. Build & Launch (Robot)
```bash
cd main_ws
just forge
just launch
```

### 4. Launch (Operator)
```bash
cd operator_ws
# Acquire security certificate from robot
just get-cert <ROBOT_IP>
# Start operator station
just launch

# (Optional) Start Foxglove Studio UI via Docker (runs on http://localhost:8080)
cd ..
just foxglove
```

## Documentation

- [Operation Manual](docs/OPERATION.md): Step-by-step guide for deployment and control.
- [System Architecture](docs/ARCHITECTURE.md): Detailed node graph, networking, and data flow.
- [Package Inventory](docs/PACKAGES.md): List of all modules and their roles.
- [Networking & Security](topology/network_topology.mmd): Mermaid diagram of the Zenoh QUIC setup.

## License
Apache-2.0
