# WT901C Current Settings & Configurator

이 문서는 WT901C의 **현재 register 설정을 읽는 방법**과 desktop `WT901C Configurator` 앱 사용법을 설명합니다.

설정값 변경 명령과 raw `FF AA ...` packet의 의미는 [`device_configuration.md`](device_configuration.md)를 함께 참고합니다.

## 1. 현재 설정을 읽을 수 있는 이유

WIT Standard Communication Protocol은 register read 명령을 제공합니다.

```text
FF AA 27 REG 00
```

- `FF AA`: host command header
- `27`: `READADDR` register command
- `REG`: 읽고 싶은 register address
- `00`: high byte

예를 들어 `BAUD (0x04)`를 읽는 요청은 다음과 같습니다.

```text
FF AA 27 04 00
```

정상 응답은 `0x55 0x5F`로 시작합니다.

```text
55 5F DATA_L DATA_H ... CHECKSUM
```

CLI는 streaming packet 사이에서 `55 5F` 응답을 찾고 checksum을 확인한 뒤 첫 register 값을 decode합니다.

---

## 2. 중요한 제약: 현재 baudrate를 알아야 read가 가능함

UART 통신은 host와 sensor baudrate가 같아야 합니다.

따라서 현재 baudrate를 모르는 상태에서 바로 `read baudrate`를 할 수는 없습니다. 먼저 지원 baudrate를 순서대로 시도해서 sensor가 응답하는 속도를 찾아야 합니다.

`wt901c_config detect-baud`는 각 baudrate에서 `VERSION (0x2E)` register를 읽고 정상 checksum 응답이 오는 값을 선택합니다.

지원 탐색 값:

```text
4800
9600
19200
38400
57600
115200
230400
```

---

## 3. Baudrate 자동 탐색

먼저 `imu_driver`를 종료합니다.

```bash
ros2 run wt901c_driver wt901c_config detect-baud
```

다른 port라면:

```bash
ros2 run wt901c_driver wt901c_config \
  --port /dev/ttyUSB1 \
  detect-baud
```

정상 예:

```text
probing=115200
detected_baudrate=115200
version=123
```

탐색 실패 시 다음을 확인합니다.

```bash
ls -l /dev/ttyUSB*
groups
```

그리고 다른 프로그램이 같은 serial port를 열고 있지 않은지 확인합니다.

---

## 4. 현재 설정 전체 읽기

현재 baudrate가 115200이라면:

```bash
ros2 run wt901c_driver wt901c_config status
```

현재 baudrate가 9600이라면:

```bash
ros2 run wt901c_driver wt901c_config \
  --baudrate 9600 \
  status
```

출력 예:

```text
connected_baudrate=115200
output_content_mask=0x001E
output_content=accel,angle,gyro,mag
output_rate_hz=100
baudrate=115200
led=on
bandwidth_hz=20
installation_direction=horizontal
algorithm=9-axis
power_on_output=on
version=123
```

`status`는 다음 register를 읽습니다.

| 표시값 | Register | Address | 의미 |
| --- | --- | ---: | --- |
| `output_content` | `RSW` | `0x02` | 어떤 `0x50~0x5A` packet을 출력할지 결정 |
| `output_rate_hz` | `RRATE` | `0x03` | sensor output rate |
| `baudrate` | `BAUD` | `0x04` | UART baudrate |
| `led` | `LEDOFF` | `0x1B` | status LED on/off |
| `bandwidth_hz` | `BANDWIDTH` | `0x1F` | internal filter bandwidth |
| `installation_direction` | `ORIENT` | `0x23` | horizontal/vertical installation |
| `algorithm` | `AXIS6` | `0x24` | 9-axis / 6-axis attitude algorithm |
| `power_on_output` | `POWONSEND` | `0x2D` | power-on streaming enable |
| `version` | `VERSION` | `0x2E` | device firmware/version register |

---

## 5. Register 하나만 읽기

이름으로 읽을 수 있습니다.

```bash
ros2 run wt901c_driver wt901c_config read baudrate
```

```bash
ros2 run wt901c_driver wt901c_config read output-rate
```

```bash
ros2 run wt901c_driver wt901c_config read output-content
```

```bash
ros2 run wt901c_driver wt901c_config read bandwidth
```

```bash
ros2 run wt901c_driver wt901c_config read install-direction
```

```bash
ros2 run wt901c_driver wt901c_config read led
```

```bash
ros2 run wt901c_driver wt901c_config read algorithm
```

```bash
ros2 run wt901c_driver wt901c_config read power-on-output
```

```bash
ros2 run wt901c_driver wt901c_config read version
```

직접 address를 지정할 수도 있습니다.

```bash
ros2 run wt901c_driver wt901c_config read 0x1F
```

예상 출력 형식:

```text
register=0x1F
raw=0x0004
decoded=20
```

`read 0xNN`에서 알려지지 않은 register는 raw 16-bit 값을 그대로 표시합니다.

---

## 6. 권장 설정 확인 순서

sensor를 처음 연결했거나 기존 설정을 모를 때:

```bash
# 1. driver 종료

# 2. 현재 baud 자동 탐색
ros2 run wt901c_driver wt901c_config detect-baud

# 3. 탐색된 baud가 115200이라고 가정
ros2 run wt901c_driver wt901c_config \
  --baudrate 115200 \
  status
```

설정을 변경한 후에는 반드시 다시 읽습니다.

```bash
ros2 run wt901c_driver wt901c_config output-rate 100
ros2 run wt901c_driver wt901c_config status
```

즉 권장 흐름은 다음과 같습니다.

```text
READ current
   ↓
CHANGE one setting
   ↓
SAVE
   ↓
READ again
   ↓
verify actual register
```

---

# WT901C Configurator GUI

## 7. 앱 목적

CLI를 외우지 않고 다음 작업을 한 화면에서 수행하기 위한 desktop app입니다.

```text
Serial port 선택
      ↓
Detect Baud
      ↓
Read Current Settings
      ↓
현재 register 값 표시
      ↓
설정 변경
      ↓
SAVE
      ↓
자동 read-back 검증
```

앱 자체가 register protocol을 별도로 재구현하지는 않습니다. 실제 sensor 통신은 같은 package의 `wt901c_config` CLI를 호출합니다.

따라서 CLI와 GUI가 서로 다른 protocol 구현을 가지지 않고 동일한 backend를 사용합니다.

---

## 8. 앱 설치/빌드

PyQt5가 필요합니다.

```bash
sudo apt update
sudo apt install python3-pyqt5
```

workspace에서:

```bash
cd ~/ros2_ws
git pull
colcon build --symlink-install --packages-up-to imu_bringup
source install/setup.bash
```

---

## 9. 앱 실행

`imu_driver`를 먼저 종료한 뒤:

```bash
ros2 run wt901c_driver wt901c_config_gui
```

앱이 열리면 다음 순서가 가장 안전합니다.

1. `Serial port` 확인
2. `Detect Baud` 클릭
3. `Read Current Settings` 클릭
4. 현재값 확인
5. 변경할 항목 하나만 선택
6. `Apply ...` 클릭
7. 앱이 자동으로 다시 read한 값을 확인

---

## 10. 앱에서 표시하는 현재값

`Current Sensor Settings` 영역에는 다음 값이 표시됩니다.

```text
Baudrate
Output rate
Output content
Output mask
Bandwidth
Installation
LED
Algorithm
Power-on output
Version
```

예:

```text
Baudrate         115200
Output rate      100
Output content   accel,angle,gyro,mag
Output mask      0x001E
Bandwidth        20
Installation     horizontal
LED              on
Algorithm        9-axis
Power-on output  on
Version          123
```

---

## 11. 앱에서 변경 가능한 설정

### Baudrate

지원:

```text
4800 / 9600 / 19200 / 38400 / 57600 / 115200 / 230400
```

Baudrate 변경 후 backend는 새 baudrate로 serial port를 다시 열고 SAVE합니다. 앱도 성공 후 자신의 `Current baud`를 새 값으로 변경한 뒤 register를 다시 읽습니다.

### Output rate

```text
0.2 / 0.5 / 1 / 2 / 5 / 10 / 20 / 50 / 100 / 200 Hz
```

### Output packets

checkbox로 선택합니다.

```text
time
accel
gyro
angle
mag
port
pressure
gps
velocity
quaternion
gsa
```

현재 driver에서 `imu/data`를 계속 사용하려면 최소한 다음이 필요합니다.

`orientation_source: angle`:

```text
accel + gyro + angle
```

`orientation_source: quaternion`:

```text
accel + gyro + quaternion
```

앱에서 accel 또는 gyro를 끄면 경고를 표시합니다.

### Bandwidth

```text
256 / 188 / 98 / 42 / 20 / 10 / 5 Hz
```

### Installation direction

```text
horizontal
vertical
```

### LED

```text
on
off
```

---

## 12. Calibration / Reference 버튼

### Set Heading Zero

현재 heading을 sensor 내부 zero reference로 설정하고 저장합니다.

CLI와 동일:

```bash
ros2 run wt901c_driver wt901c_config heading-zero
```

### Start Magnetic Calibration

```bash
ros2 run wt901c_driver wt901c_config mag-calibration start
```

센서를 다양한 자세로 천천히 회전합니다.

### Stop & Save Magnetic Calibration

```bash
ros2 run wt901c_driver wt901c_config mag-calibration stop
```

normal mode로 돌아온 뒤 결과를 저장합니다.

---

## 13. 앱의 Log 영역

앱 하단 Log에는 실제 실행된 backend command와 output/error가 표시됩니다.

예:

```text
$ .../wt901c_config --port /dev/ttyUSB0 --baudrate 115200 status
connected_baudrate=115200
output_content_mask=0x001E
...
```

따라서 GUI에서 문제가 발생해도 어떤 CLI command가 실패했는지 바로 확인할 수 있습니다.

---

## 14. `imu_driver`와 Configurator를 동시에 실행하지 않는 이유

둘 다 같은 UART device를 사용합니다.

```text
imu_driver --------┐
                   ├── /dev/ttyUSB0   ← 충돌 가능
configurator ------┘
```

권장:

```text
configuration 작업
  driver OFF
  configurator ON

normal operation
  configurator OFF
  driver ON
```

설정이 끝난 뒤 앱을 닫고:

```bash
ros2 launch imu_bringup imu_bringup.launch.py
```

으로 driver를 다시 시작합니다.

---

## 15. 현재 read 기능의 범위

현재 GUI/status는 운용 중 자주 확인할 주요 configuration register를 우선 지원합니다.

추가 register도 protocol상 read 가능하며 CLI에서 address를 직접 지정할 수 있습니다.

```bash
ros2 run wt901c_driver wt901c_config read 0xNN
```

다만 모든 register를 GUI에 노출하지 않은 이유는 다음과 같습니다.

- 모델별로 의미가 다른 register가 있음
- 잘못 수정하면 sensor algorithm 동작이 달라질 수 있음
- 일반 운용에서 필요 없는 calibration/internal filter register가 많음

따라서 GUI는 **자주 사용하고 의미가 명확한 설정만 write 가능**하게 유지하고, 추가 register는 read부터 검증한 뒤 필요할 때 명시적으로 지원하는 것을 권장합니다.
