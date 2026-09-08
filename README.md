# f1tenth_system

Drivers onboard f1tenth race cars. This branch is under development for migration to ROS2. See the [documentation of F1TENTH](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html) on how to get started.

## Deadman's switch
On Logitech F-710 joysticks, the LB button is the deadman's switch for teleop, and the RB button is the deadman's switch for navigation. You can also remap buttons. See how on the readthedocs documentation.

## Topics

### Topics that the driver stack subscribe to
- `/drive`: Topic for autonomous navigation, uses `AckermannDriveStamped` messages.

### Sensor topics published by the driver stack
- `/scan`: Topic for `LaserScan` messages.
- `/odom`: Topic for `Odometry` messages.
- `/sensors/imu/raw`: Topic for `Imu` messages.
- `/sensors/core`: Topic for telemetry data from the VESC

## External Dependencies

1. ackermann_msgs [https://index.ros.org/r/ackermann_msgs/#foxy](https://index.ros.org/r/ackermann_msgs/#foxy).
2. urg_node [https://index.ros.org/p/urg_node/#foxy](https://index.ros.org/p/urg_node/#foxy). This is the driver for Hokuyo LiDARs.
3. joy [https://index.ros.org/p/joy/#foxy](https://index.ros.org/p/joy/#foxy). This is the driver for joysticks in ROS 2.
4. teleop_tools  [https://index.ros.org/p/teleop_tools/#foxy](https://index.ros.org/p/teleop_tools/#foxy). This is the package for teleop with joysticks in ROS 2.
5. vesc [GitHub - f1tenth/vesc at ros2](https://github.com/f1tenth/vesc/tree/ros2). This is the driver for VESCs in ROS 2.
6. ackermann_mux [GitHub - f1tenth/ackermann_mux: Twist multiplexer](https://github.com/f1tenth/ackermann_mux). This is a package for multiplexing ackermann messages in ROS 2.
<!-- 7. rosbridge_suite [https://index.ros.org/p/rosbridge_suite/#foxy-overview](https://index.ros.org/p/rosbridge_suite/#foxy-overview) This is a package that allows for websocket connection in ROS 2. -->

## Package in this repo

1. f1tenth_stack: maintains the bringup launch and all parameter files

## Nodes launched in bringup

1. joy
2. joy_teleop
3. ackermann_to_vesc_node
4. vesc_to_odom_node
5. vesc_driver_node
6. urg_node
7. ackermann_mux

## Parameters and topics for dependencies

### vesc_driver

1. Parameters:
   - duty_cycle_min, duty_cycle_max
   - current_min, current_max
   - brake_min, brake_max
   - speed_min, speed_max
   - position_min, position_max
   - servo_min, servo_max
2. Publishes to:
   - sensors/core
   - sensors/servo_position_command
   - sensors/imu
   - sensors/imu/raw
3. Subscribes to:
   - commands/motor/duty_cycle
   - commands/motor/current
   - commands/motor/brake
   - commands/motor/speed
   - commands/motor/position
   - commands/servo/position

### ackermann_to_vesc

1. Parameters:
   - speed_to_erpm_gain
   - speed_to_erpm_offset
   - steering_angle_to_servo_gain
   - steering_angle_to_servo_offset
2. Publishes to:
   - ackermann_cmd
3. Subscribes to:
   - commands/motor/speed
   - commands/servo/position

### vesc_to_odom

1. Parameters:
   - odom_frame
   - base_frame
   - use_servo_cmd_to_calc_angular_velocity
   - speed_to_erpm_gain
   - speed_to_erpm_offset
   - steering_angle_to_servo_gain
   - steering_angle_to_servo_offset
   - wheelbase
   - publish_tf
2. Publishes to:
   - odom
3. Subscribes to:
   - sensors/core
   - sensors/servo_position_command

### throttle_interpolator

1. Parameters:
   - rpm_input_topic
   - rpm_output_topic
   - servo_input_topic
   - servo_output_topic
   - max_acceleration
   - speed_max
   - speed_min
   - throttle_smoother_rate
   - speed_to_erpm_gain
   - max_servo_speed
   - steering_angle_to_servo_gain
   - servo_smoother_rate
   - servo_max
   - servo_min
   - steering_angle_to_servo_offset
2. Publishes to:
   - topic described in rpm_output_topic
   - topic described in servo_output_topic
3. Subscribes to:
   - topic described in rpm_input_topic
   - topic described in servo_input_topic
  

# F1TENTH ROS2 Safety Controller

LiDAR 데이터를 이용해 전방 장애물을 감지하고 F1TENTH 차량의 속도를 제어하는 ROS 2 프로젝트입니다. 
차량에 탑재된 ROS 2 Foxy 환경을 기준으로 안전 정지 노드와 조이스틱 데드맨 스위치 기반 주행 노드를 구성했습니다.

## 주요 기능

- `LaserScan`의 전방 ±15° 데이터를 필터링해 가장 가까운 장애물 거리 계산.
- 장애물 거리에 따라 정지, 저속, 정상 주행의 3단계 속도 명령 생성.
- 유효한 LiDAR 데이터가 없으면 차량을 정지시키는 fail-safe 동작 적용.
- 조이스틱 RB 버튼을 데드맨 스위치로 사용해 버튼을 놓는 즉시 정지 명령 발행.
- `AckermannDriveStamped` 메시지로 차량 구동 명령 전달.

## 동작 구조

```text
YDLiDAR ── /scan ──> safety_node ── /drive ──> ackermann_mux ──> VESC
Joystick ── /joy ──> ttc_driver_with_joy ────────┘
```

### 거리 기반 속도 정책

| 전방 장애물 거리 | 목표 속도 |
| --- | ---: |
| 0.5 m 미만 | 0.0 m/s |
| 0.5 m 이상 1.5 m 미만 | 0.5 m/s |
| 1.5 m 이상 | 최대 1.5 m/s |
| 유효한 센서 데이터 없음 | 0.0 m/s |

임계값과 속도는 ROS 2 파라미터로 조정할 수 있습니다.

## 기술 스택

- ROS 2 Foxy
- Python 3
- NVIDIA Jetson Nano
- YDLiDAR
- VESC, Ackermann steering
- `colcon`, `ament_cmake`

## 프로젝트 구조

```text
f1tenth_ws/src/f1tenth_system/
├── safety_node/       # LiDAR 기반 장애물 감지 및 안전 속도 제어
├── f1tenth_stack/     # 차량 bring-up, 조이스틱 및 구동 설정
├── ackermann_mux/     # 주행 명령 우선순위 처리
└── teleop_tools/      # 키보드·마우스·조이스틱 원격 조작 도구
```

## 빌드 및 실행

ROS 2 Foxy와 프로젝트 의존 패키지가 설치된 환경에서 실행.

```bash
cd f1tenth_ws
colcon build --symlink-install
source install/setup.bash
```

LiDAR 안전 제어 노드:

```bash
ros2 run safety_node safety_node.py
```

조이스틱 데드맨 스위치 기반 주행 노드:

```bash
ros2 run f1tenth_stack ttc_driver_with_joy
```

## ROS 인터페이스

| 구분 | 토픽 | 메시지 |
| --- | --- | --- |
| Subscribe | `/scan` | `sensor_msgs/LaserScan` |
| Subscribe | `/joy` | `sensor_msgs/Joy` |
| Publish | `/drive` | `ackermann_msgs/AckermannDriveStamped` |

## 테스트

센서 데이터 필터링과 속도 결정 로직은 ROS 2 실행 환경 없이도 단위 테스트할 수 있습니다.

```bash
cd f1tenth_ws/src/f1tenth_system/safety_node
python3 -m unittest discover -s tests
```

ROS 2 환경에서는 워크스페이스 루트에서 다음 명령으로 같은 테스트를 실행할 수 있습니다.

```bash
colcon test --packages-select safety_node
```

## 참고 및 출처

이 저장소는 오픈소스 [F1TENTH system](https://github.com/f1tenth/f1tenth_system)을 기반으로 구성했으며, 포함된 외부 패키지의 라이선스와 저작권 표시는 각 디렉터리에 유지했습니다.
