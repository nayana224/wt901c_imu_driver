# WT901C ROS 2 Driver

ROS 2 Humble driver for the WitMotion WT901C UART/TTL IMU/AHRS sensor.

The driver validates and resynchronizes the WT901C 11-byte stream, converts sensor values to ROS SI units, publishes IMU/temperature/magnetic-field messages, reconnects after serial failures, and exposes non-blocking accelerometer calibration.

## Supported environment

- Ubuntu 22.04
- ROS 2 Humble
- WT901C TTL/UART
- Default example: `/dev/ttyUSB0`, 115200 baud

The implementation uses the ROS 2 compatible `serial` package that provides `serial/serial.h` and `serialConfig.cmake`. This is **not** the same API as ROS 2 `serial_driver`.

One compatible source package is:

```bash
cd ~/ros2_ws/src
git clone https://github.com/RoverRobotics-forks/serial-ros2.git serial
```

## Workspace setup

```bash
cd ~/ros2_ws/src
git clone https://github.com/nayana224/wt901c_imu_driver.git
cd ..

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to imu_bringup
source install/setup.bash
```

For serial access, prefer the `dialout` group instead of making the device world-writable:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing the group membership.

## Run

Recommended bringup:

```bash
ros2 launch imu_bringup imu_bringup.launch.py
```

Override hardware settings when needed:

```bash
ros2 launch imu_bringup imu_bringup.launch.py \
  port:=/dev/ttyUSB1 \
  baudrate:=115200 \
  frame_id:=imu_link
```

Run the driver directly:

```bash
ros2 run wt901c_driver imu_driver --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200
```

## ROS interfaces

### Published topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `imu/data` | `sensor_msgs/msg/Imu` | Orientation, angular velocity, linear acceleration |
| `imu/temperature` | `sensor_msgs/msg/Temperature` | Sensor temperature from the `0x51` packet |
| `imu/mag` | `sensor_msgs/msg/MagneticField` | Magnetic field converted to Tesla |

The publisher uses `rclcpp::SensorDataQoS`.

### Service

| Service | Type | Meaning |
| --- | --- | --- |
| `calibrate_imu` | `std_srvs/srv/Empty` | Starts WT901C accelerometer calibration |

Calibration is asynchronous. The service callback returns immediately; keep the sensor horizontal and completely still until the node logs that calibration has completed and the configuration has been saved.

```bash
ros2 service call /calibrate_imu std_srvs/srv/Empty "{}"
```

The command sequence follows the WIT register protocol:

```text
unlock (0x69 = 0xB588)
-> CALSW = 0x01
-> wait
-> CALSW = 0x00
-> SAVE = 0x0000
```

## Parameters

Default parameters are in [`src/imu_bringup/config/wt901c.yaml`](src/imu_bringup/config/wt901c.yaml).

| Parameter | Default | Description |
| --- | ---: | --- |
| `port` | `/dev/ttyUSB0` | Serial device |
| `baudrate` | `115200` | UART baud rate |
| `frame_id` | `imu_link` | `header.frame_id` for sensor messages |
| `orientation_source` | `angle` | `angle` for `0x53`, or `quaternion` for `0x59` |
| `yaw_offset_rad` | `0.0` | Extra yaw alignment applied before publishing orientation |
| `poll_interval_ms` | `2` | Serial polling period |
| `serial_timeout_ms` | `20` | Serial library timeout |
| `reconnect_interval_ms` | `1000` | Delay between reconnect attempts |
| `calibration_duration_seconds` | `5.0` | Accelerometer calibration wait time |
| `magnetic_tesla_per_lsb` | `6.67e-9` | Magnetic-field scale factor |
| `temperature_variance` | `0.0` | Temperature variance; zero means unknown |
| `*_covariance_diagonal` | `[0, 0, 0]` | Per-axis covariance; zero means unknown |

Do not invent covariance values for filtering. Measure stationary sensor noise in the actual mounting/environment and configure the resulting variances.

## Protocol behavior

The WT901C active output frame is 11 bytes:

```text
0x55 | TYPE | DATA0 ... DATA7 | CHECKSUM
```

The driver supports:

- `0x51`: acceleration and temperature
- `0x52`: angular velocity
- `0x53`: Euler angles
- `0x54`: magnetic field
- `0x59`: quaternion

Incoming bytes are buffered and scanned for `0x55`. If a checksum fails, only the candidate header byte is discarded, allowing the parser to resynchronize on the next valid frame.

An `imu/data` message is published only after one coherent sample has received:

```text
0x51 acceleration
-> 0x52 angular velocity
-> selected orientation packet (0x53 or 0x59)
```

This prevents a dropped packet from silently mixing a new orientation with acceleration or angular velocity left over from the previous sample.

### Units

- Linear acceleration: `m/s^2`
- Angular velocity: `rad/s`
- Orientation: quaternion
- Magnetic field: `Tesla`
- Temperature: `degC`

The conversion follows the WIT protocol scales (`±16 g`, `±2000 deg/s`, angle `±180 deg`, quaternion `/32768`).

## Orientation and TF

The WT901C manual describes the sensor body axes as:

```text
x: forward
y: left
z: up
```

Those body axes match the ROS body-frame convention. The sensor's absolute heading reference, however, comes from the WIT attitude solution and magnetic reference. The driver does **not** guess an application-specific geographic ENU heading conversion. Use `yaw_offset_rad` or your localization/fusion layer when a known world-heading alignment is required.

The driver intentionally does **not** publish `world -> imu_link` TF. On a robot, the physical mounting transform should normally come from URDF or another static transform, for example:

```text
base_link -> imu_link
```

`view_imu_test.launch.py` creates a temporary `world -> imu_link` static transform only for bench visualization.

## Quaternion output

Factory output commonly includes acceleration, angular velocity, angle, and magnetic field, but not `0x59` quaternion output. Keep:

```yaml
orientation_source: angle
```

unless quaternion output has been enabled in the WT901C return-content (`RSW`) configuration. If `orientation_source: quaternion` is selected without enabling `0x59`, `imu/data` will not be published and the driver will warn about incomplete samples.

WIT defines quaternion packet order as:

```text
q0, q1, q2, q3 = w, x, y, z
```

## Serial failure behavior

The node does not terminate just because the USB-UART device is temporarily unavailable. It:

1. catches serial open/read/write failures,
2. clears partial packet/sample state,
3. aborts an in-progress calibration if needed,
4. retries the configured serial device after `reconnect_interval_ms`.

This allows unplug/replug recovery without restarting the ROS 2 process, assuming the device returns at the same path.

## Package structure

```text
src/
├── wt901c_driver/
│   ├── include/wt901c_driver/
│   │   ├── imu_driver.hpp
│   │   └── protocol.hpp
│   ├── src/
│   │   ├── imu_driver.cpp
│   │   └── protocol.cpp
│   └── test/
│       └── test_protocol.cpp
└── imu_bringup/
    ├── config/
    │   └── wt901c.yaml
    └── launch/
        ├── imu_bringup.launch.py
        └── view_imu_test.launch.py
```

`wt901c_driver` owns sensor communication and ROS sensor interfaces. `imu_bringup` owns deployment configuration and launch composition only.

## Validation

Protocol tests cover checksum/resynchronization and core unit conversions:

```bash
colcon test --packages-select wt901c_driver
colcon test-result --verbose
```

For hardware validation, verify at minimum:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /imu/temperature --once
ros2 topic echo /imu/mag --once
```

Then check:

- stationary acceleration magnitude is approximately `9.81 m/s^2`,
- rotation sign follows the physical sensor axes,
- unplug/replug recovers automatically,
- calibration completes without blocking the ROS executor,
- `frame_id` matches the robot URDF/static TF.

## Reference manuals

The repository contains the vendor documents used for protocol verification:

- `WIT Standard Communication Protocol.pdf`
- `WT901C TTL Manual.pdf`

The source also follows ROS SI-unit and body-frame conventions; global heading alignment should be validated for the actual robot and localization setup.
