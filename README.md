# WT901C ROS2 Humble Driver

## 1. Introduction

This package is a dedicated ROS2 Humble driver for the **WitMotion WT901C IMU sensor**. It parses raw 16-bit serial data into standard ROS2 messages, providing high-precision orientation, acceleration, and angular velocity for robotic applications.

## 2. Technical Specifications

### 2.1 ROS2 API (Nodes & Interfaces)

| Interface     | Name        | Type                            | Description                                                |
| :------------ | :---------- | :------------------------------ | :--------------------------------------------------------- |
| **Topic**     | `/imu/data` | `sensor_msgs/msg/Imu`           | Fused orientation (Quaternion) and inertial data.          |
| **Topic**     | `/imu/mag`  | `sensor_msgs/msg/MagneticField` | Raw 3-axis magnetic field strength.                        |
| **Parameter** | `port_name` | `string`                        | Serial device path (Default: `/dev/ttyUSB0`).              |
| **Parameter** | `baudrate`  | `int`                           | Communication speed (Default: `115200`).                   |
| **Parameter** | `frame_id`  | `string`                        | TF frame associated with the sensor (Default: `imu_link`). |

### 2.2 Hardware Setup

- **Communication**: UART (TTL Level)
- **Baudrate**: Supported up to 115200 bps.
- **Update Rate**: Up to 200Hz.

## 3. Installation & Build

### Prerequisites

Ensure you have a ROS2 Humble environment and the `serial` library installed.

```bash
# Update and install dependencies
sudo apt update
sudo apt install ros-humble-serial-driver
```

### Build Instructions

```
cd ~/ros2_ws/src
git clone <repository_url>
cd ..
colcon build --packages-select wt901c_driver
source install/setup.bash
```

## 4. Usage

To launch the driver with default parameters:

```
ros2 launch wt901c_driver wt901c_launch.py
```
