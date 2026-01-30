# ZeusCar ROS2 Jazzy Basic

A ROS2 Jazzy-based robot car control system for the ZeusCar platform running on Raspberry Pi 4.

## Overview

This project migrates the original ZeusCar robot control system from Ubuntu 22.04 + ROS2 Humble to Ubuntu 24.04 LTS + ROS2 Jazzy Jalisco, while maintaining full functional compatibility with the existing hardware setup.

## Features

- **11-Direction Movement Control**: Forward, backward, lateral movement, diagonal movement, and rotation
- **LiDAR Integration**: SLAMTEC RPLIDAR support for environment scanning
- **Arduino Communication**: Serial interface to Arduino Uno R3 for motor control
- **ROS2 Topic-based Control**: Standard message-based command interface

## System Architecture

```
┌─────────────────┐
│   Host PC       │
│  (Publisher)    │
└────────┬────────┘
         │ ROS2 Topic: "topic"
         │ (std_msgs/String)
         ▼
┌─────────────────┐
│ Raspberry Pi 4  │
│  (Subscriber)   │
│  (sllidar_node) │
└────────┬────────┘
         │ Serial: /dev/ttyACM0 (9600 baud)
         ▼
┌─────────────────┐
│  Arduino Uno R3 │
└────────┬────────┘
         │ PWM Control
         ▼
┌─────────────────┐
│  DC Motors x4   │
└─────────────────┘
```

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Main Board | Raspberry Pi 4 |
| Microcontroller | Arduino Uno R3 |
| Sensor | SLAMTEC RPLIDAR (A1/A2/A3/S1/S2/etc.) |
| Motors | DC Motor x 4 (Mecanum wheels) |

## Software Requirements

| Component | Version |
|-----------|---------|
| OS | Ubuntu 24.04.3 LTS (Noble Numbat) |
| ROS2 | Jazzy Jalisco |
| Python | 3.x |
| C++ | C++17 |

## Project Structure

```
zeuscar-ros2-jazzy-basic/
├── src/
│   ├── zeuscar_robot_package/    # Robot control package (Python)
│   ├── robot_description/        # URDF/Xacro robot definition
│   └── sllidar_ros2/             # LiDAR driver package
├── docs/                         # Documentation
├── reference/                    # Reference files (read-only)
├── CLAUDE.md                     # Claude Code configuration
└── LICENSE
```

## ROS2 Packages

| Package | Description | Build Type |
|---------|-------------|------------|
| zeuscar_robot_package | Robot control subscriber node | ament_python |
| robot_description | URDF robot model | ament_cmake |
| sllidar_ros2 | LiDAR driver | ament_cmake |

## Installation

### Prerequisites

1. Install Ubuntu 24.04 LTS on Raspberry Pi 4
2. Install ROS2 Jazzy Jalisco

### Clone and Build

```bash
# Clone the repository
git clone https://github.com/your-username/zeuscar-ros2-jazzy-basic.git
cd zeuscar-ros2-jazzy-basic

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Start the Subscriber Node (Robot Control)

```bash
ros2 run zeuscar_robot_package subscriber_node
```

### Start the LiDAR Node

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```

### Visualize in RViz

```bash
ros2 launch robot_description display.launch.py
```

### Send Movement Commands

```bash
ros2 topic pub /topic std_msgs/String "data: 'FORWARD'"
```

## Movement Commands

| Command | Action |
|---------|--------|
| FORWARD | Move forward |
| BACKWARD | Move backward |
| LEFT | Strafe left |
| RIGHT | Strafe right |
| LEFTFORWARD | Move diagonally left-forward |
| RIGHTFORWARD | Move diagonally right-forward |
| LEFTBACKWARD | Move diagonally left-backward |
| RIGHTBACKWARD | Move diagonally right-backward |
| TURNLEFT | Rotate left |
| TURNRIGHT | Rotate right |
| STOP | Stop all motors |

## Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| topic | std_msgs/String | Movement commands |
| /scan | sensor_msgs/LaserScan | LiDAR scan data |

## Documentation

Detailed documentation is available in the `docs/` directory:

- [Requirements](docs/requirements.md) - Project requirements
- [System Specification](docs/system-specification.md) - Technical specifications
- [Setup Guide (Raspberry Pi)](docs/setup-guide.md) - Installation guide for Raspberry Pi
- [Setup Guide (Host PC)](docs/setup-guide-host-pc.md) - Installation guide for Host PC
- [Troubleshooting](docs/troubleshooting.md) - Common issues and solutions

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

Copyright (c) 2026 Murasan
