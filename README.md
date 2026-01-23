# WT901C ROS2 Humble Driver

ROS2 Humble Driver for WitMotion WT901C IMU Sensor.

## 1. Prerequisites

- ROS2 Humble
- [serial](https://github.com/wjwwood/serial) library

```bash
# Install dependencies
sudo apt update
sudo apt install ros-humble-serial-driver
```

## 2. Hardware Setup

- ​**Interface**​: UART (TTL Level)
- ​**Default Baudrate**​: 115200
- ​**Default Port**​: `/dev/ttyUSB0`
- ​**Pinout**​: VCC (3.3-5V), GND, TX, RX

## 3. Installation

```
cd ~/ros2_ws/src
git clone <repository_url>
cd ..
colcon build --packages-select wt901c_driver
source install/setup.bash
```

## 4. Usage

```
# Ensure serial permission
sudo chmod 666 /dev/ttyUSB0

# Run the driver
ros2 run wt901c_driver imu_driver
```

## 5. ROS2 API

### Published Topics

- `/imu/data` (`sensor_msgs/msg/Imu`): Fused orientation, angular velocity, and linear acceleration.

### Services

- `calibrate_imu` (`std_srvs/srv/Empty`): Initiates acceleration and gyroscope zero-point calibration.

### TF

- `world` -> `imu_link`: Real-time orientation broadcast.

### Parameters (Hardcoded in Source)

- Port: `/dev/ttyUSB0`
- Baudrate: 115200
