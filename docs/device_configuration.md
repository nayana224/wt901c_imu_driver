# WT901C Device Configuration

이 문서는 `wt901c_config` CLI로 WT901C의 UART 설정과 출력 설정을 변경하는 방법을 정리합니다.

설정 값은 WIT Standard Communication Protocol의 register 정의를 기준으로 합니다. 저장소 루트의 `WIT Standard Communication Protocol.pdf`와 `WT901C TTL Manual.pdf`도 함께 참고합니다.

## 1. 사용 전 원칙

장치 설정 중에는 `imu_driver`를 중지합니다. 두 프로세스가 같은 serial port를 동시에 열거나 명령과 streaming data를 동시에 처리하지 않도록 하기 위함입니다.

```bash
# bringup이 실행 중이면 Ctrl+C로 먼저 종료
```

기본 장치가 `/dev/ttyUSB0`, 현재 baudrate가 `115200`이면 다음 형식을 사용합니다.

```bash
ros2 run wt901c_driver wt901c_config COMMAND [ARGS...]
```

다른 port 또는 **현재 센서 baudrate**를 사용할 때는 앞에 옵션을 붙입니다.

```bash
ros2 run wt901c_driver wt901c_config \
  --port /dev/ttyUSB1 \
  --baudrate 9600 \
  COMMAND [ARGS...]
```

`--baudrate`는 바꾸려는 값이 아니라 **명령을 보내기 전에 센서가 현재 사용 중인 baudrate**입니다.

실제 송신 전 command bytes를 확인하려면 `--dry-run`을 사용합니다.

```bash
ros2 run wt901c_driver wt901c_config --dry-run output-rate 100
```

WIT write command의 기본 형식은 다음과 같습니다.

```text
FF AA REG DATA_L DATA_H
```

설정을 바꿀 때 도구는 일반적으로 다음 순서로 보냅니다.

```text
unlock
  FF AA 69 88 B5

write register
  FF AA REG DATA_L DATA_H

save
  FF AA 00 00 00
```

WIT protocol은 register write 전에 unlock을 요구하며, 변경 내용을 비휘발성 설정으로 남기려면 SAVE를 사용합니다.

---

## 2. Baudrate 변경

### 명령

```bash
ros2 run wt901c_driver wt901c_config baudrate 9600
```

현재 sensor baudrate가 115200이 아니라면 반드시 현재 값을 지정합니다.

```bash
ros2 run wt901c_driver wt901c_config \
  --baudrate 9600 \
  baudrate 115200
```

지원 값:

| Baudrate | BAUD value | Register command |
| ---: | ---: | --- |
| 4800 | `0x01` | `FF AA 04 01 00` |
| 9600 | `0x02` | `FF AA 04 02 00` |
| 19200 | `0x03` | `FF AA 04 03 00` |
| 38400 | `0x04` | `FF AA 04 04 00` |
| 57600 | `0x05` | `FF AA 04 05 00` |
| 115200 | `0x06` | `FF AA 04 06 00` |
| 230400 | `0x07` | `FF AA 04 07 00` |

WT901C에서는 4800~230400 bps 범위만 이 CLI에서 허용합니다. Protocol 문서에 있는 460800/921600은 WT931/JY931/HWT606/HWT906 계열용으로 표시되어 있으므로 WT901C에서는 막았습니다.

### 실제 내부 순서

예를 들어 115200 → 9600 변경 시:

```text
host open @ 115200
FF AA 69 88 B5    unlock
FF AA 04 02 00    BAUD = 9600

sensor UART changes to 9600

host reopen @ 9600
FF AA 69 88 B5    unlock
FF AA 00 00 00    SAVE
```

Baudrate를 바꾼 뒤에는 bringup 설정도 반드시 맞춥니다.

```yaml
# src/imu_bringup/config/wt901c.yaml
baudrate: 9600
```

또는 launch에서 덮어씁니다.

```bash
ros2 launch imu_bringup imu_bringup.launch.py baudrate:=9600
```

baudrate가 서로 다르면 sensor packet을 정상적으로 읽을 수 없습니다.

---

## 3. Output rate 변경

Register: `RRATE (0x03)`

```bash
ros2 run wt901c_driver wt901c_config output-rate 100
```

지원 값:

| Rate | RRATE | Raw command |
| ---: | ---: | --- |
| 0.2 Hz | `0x01` | `FF AA 03 01 00` |
| 0.5 Hz | `0x02` | `FF AA 03 02 00` |
| 1 Hz | `0x03` | `FF AA 03 03 00` |
| 2 Hz | `0x04` | `FF AA 03 04 00` |
| 5 Hz | `0x05` | `FF AA 03 05 00` |
| 10 Hz | `0x06` | `FF AA 03 06 00` |
| 20 Hz | `0x07` | `FF AA 03 07 00` |
| 50 Hz | `0x08` | `FF AA 03 08 00` |
| 100 Hz | `0x09` | `FF AA 03 09 00` |
| 200 Hz | `0x0B` | `FF AA 03 0B 00` |

예:

```bash
ros2 run wt901c_driver wt901c_config output-rate 200
```

변경 후 확인:

```bash
ros2 launch imu_bringup imu_bringup.launch.py
```

다른 terminal:

```bash
ros2 topic hz /imu/data
```

주의: `/imu/data`의 실제 publish rate는 sensor에서 필요한 `0x51 + 0x52 + orientation` packet 조합이 정상적으로 도착해야 하므로 장치 설정과 host 처리 상태의 영향을 받습니다.

---

## 4. Output content 변경

Register: `RSW (0x02)`

```bash
ros2 run wt901c_driver wt901c_config \
  output-content accel,gyro,angle,mag
```

사용 가능한 이름:

| Name | Bit | Sensor frame |
| --- | ---: | --- |
| `time` | 0 | `0x50` |
| `accel` | 1 | `0x51` |
| `gyro` | 2 | `0x52` |
| `angle` | 3 | `0x53` |
| `mag` | 4 | `0x54` |
| `port` | 5 | `0x55` |
| `pressure` | 6 | `0x56` |
| `gps` | 7 | `0x57` |
| `velocity` | 8 | `0x58` |
| `quaternion` | 9 | `0x59` |
| `gsa` | 10 | `0x5A` |

예를 들어:

```bash
ros2 run wt901c_driver wt901c_config \
  output-content accel,gyro,angle,mag
```

mask는 다음과 같습니다.

```text
MAG   bit4 = 0x10
ANGLE bit3 = 0x08
GYRO  bit2 = 0x04
ACC   bit1 = 0x02
-----------------
mask         0x1E
```

실제 register write:

```text
FF AA 02 1E 00
```

### `imu/data`를 사용할 때 필수 조합

현재 driver는 stale sample을 섞지 않기 위해 하나의 IMU sample에 다음 세 종류가 모두 필요합니다.

`orientation_source: angle`인 경우:

```text
accel + gyro + angle
```

권장:

```bash
ros2 run wt901c_driver wt901c_config \
  output-content accel,gyro,angle,mag
```

`orientation_source: quaternion`인 경우:

```text
accel + gyro + quaternion
```

예:

```bash
ros2 run wt901c_driver wt901c_config \
  output-content accel,gyro,quaternion,mag
```

그리고 config도 맞춥니다.

```yaml
orientation_source: quaternion
```

필수 packet이 빠진 output mask를 요청하면 CLI가 경고를 출력합니다.

---

## 5. Bandwidth 변경

Register: `BANDWIDTH (0x1F)`

```bash
ros2 run wt901c_driver wt901c_config bandwidth 20
```

| Bandwidth | Value |
| ---: | ---: |
| 256 Hz | `0x00` |
| 188 Hz | `0x01` |
| 98 Hz | `0x02` |
| 42 Hz | `0x03` |
| 20 Hz | `0x04` |
| 10 Hz | `0x05` |
| 5 Hz | `0x06` |

예:

```text
bandwidth 20
→ FF AA 1F 04 00
```

Bandwidth는 sensor 내부 filtering/응답 특성에 영향을 주므로 숫자가 높다고 항상 좋은 것은 아닙니다. 실제 robot vibration과 원하는 response speed를 기준으로 결정합니다.

---

## 6. 현재 heading을 0°로 설정

Register: `CALSW (0x01)`, value `0x04`

```bash
ros2 run wt901c_driver wt901c_config heading-zero
```

주요 packet:

```text
FF AA 01 04 00
```

현재 sensor heading을 zero reference로 설정합니다.

이 기능은 ROS의 `yaw_offset_rad`와 역할이 다릅니다.

- `heading-zero`: **sensor 내부 reference 자체를 변경**
- `yaw_offset_rad`: sensor data는 그대로 두고 **ROS publish orientation에 software offset 적용**

실험 기준을 반복해서 재현해야 한다면 둘을 섞어 사용하지 말고 하나의 기준 방식을 정하는 것을 권장합니다.

---

## 7. 설치 방향 설정

Register: `ORIENT (0x23)`

Horizontal:

```bash
ros2 run wt901c_driver wt901c_config install-direction horizontal
```

```text
FF AA 23 00 00
```

Vertical:

```bash
ros2 run wt901c_driver wt901c_config install-direction vertical
```

```text
FF AA 23 01 00
```

Vertical mode는 protocol에서 sensor coordinate의 Y-axis arrow가 위쪽을 향하는 설치 조건으로 설명됩니다. 실제 robot의 `imu_link` 축은 URDF와 sensor 장착 방향을 함께 검증해야 합니다.

---

## 8. LED 설정

Register: `LEDOFF (0x1B)`

LED on:

```bash
ros2 run wt901c_driver wt901c_config led on
```

```text
FF AA 1B 00 00
```

LED off:

```bash
ros2 run wt901c_driver wt901c_config led off
```

```text
FF AA 1B 01 00
```

---

## 9. Magnetic-field calibration

Spherical fitting calibration을 시작합니다.

```bash
ros2 run wt901c_driver wt901c_config mag-calibration start
```

주요 command:

```text
FF AA 01 07 00
```

그 상태에서 sensor를 여러 방향으로 천천히 회전시켜 충분한 orientation을 수집합니다. 주변 철 구조물, motor, 자석, 큰 전류가 흐르는 cable에서 가능한 한 떨어진 실제 사용 환경에서 수행합니다.

완료 후:

```bash
ros2 run wt901c_driver wt901c_config mag-calibration stop
```

이 명령은 normal mode로 복귀하고 설정을 저장합니다.

```text
FF AA 01 00 00
FF AA 00 00 00
```

---

## 10. SAVE만 명시적으로 보내기

```bash
ros2 run wt901c_driver wt901c_config save
```

주요 command:

```text
FF AA 00 00 00
```

일반적인 설정 command는 이미 마지막에 SAVE까지 수행하므로 보통 따로 실행할 필요는 없습니다.

---

## 11. `--dry-run`으로 먼저 확인하기

실제 설정을 바꾸기 전에 권장합니다.

```bash
ros2 run wt901c_driver wt901c_config \
  --dry-run \
  output-content accel,gyro,angle,mag
```

예상 형태:

```text
[dry-run] open /dev/ttyUSB0 @ 115200 bps
[dry-run] TX: FF AA 69 88 B5
[dry-run] TX: FF AA 02 1E 00
[dry-run] TX: FF AA 00 00 00
Output content mask set to 0x001E.
```

Baudrate 변경도 dry-run으로 sequence를 확인할 수 있습니다.

```bash
ros2 run wt901c_driver wt901c_config \
  --dry-run \
  baudrate 9600
```

---

## 12. 권장 WT901C 설정 예

현재 driver의 기본 `orientation_source: angle`을 사용할 때:

```bash
# driver는 중지한 상태

ros2 run wt901c_driver wt901c_config output-rate 100

ros2 run wt901c_driver wt901c_config \
  output-content accel,gyro,angle,mag

ros2 run wt901c_driver wt901c_config bandwidth 20
```

그 후:

```bash
ros2 launch imu_bringup imu_bringup.launch.py
ros2 topic hz /imu/data
```

설정을 하나씩 바꾸고 결과를 확인하는 것을 권장합니다. 여러 register를 동시에 바꾼 뒤 문제가 생기면 원인을 구분하기 어렵습니다.

---

## 13. 문제 해결

### 설정 도구가 port를 열지 못함

먼저 driver가 실행 중인지 확인합니다.

```bash
ros2 node list
```

그리고 serial device와 권한을 확인합니다.

```bash
ls -l /dev/ttyUSB*
groups
```

### Baudrate 변경 후 sensor data가 안 나옴

가장 흔한 원인은 host와 sensor baudrate 불일치입니다.

예를 들어 sensor를 9600으로 바꿨다면:

```bash
ros2 launch imu_bringup imu_bringup.launch.py baudrate:=9600
```

### `/imu/data`가 publish되지 않음

output content에 다음 packet이 있는지 확인합니다.

```text
accel
gyro
angle 또는 quaternion
```

`orientation_source`도 sensor output과 일치해야 합니다.

### 설정을 바꾸기 전에 bytes를 확인하고 싶음

항상 `--dry-run`을 먼저 사용합니다.

---

## References

- `WIT Standard Communication Protocol.pdf`
- `WT901C TTL Manual.pdf`
- WITMOTION, WIT Standard Communication Protocol
- WITMOTION WT901C product specifications
