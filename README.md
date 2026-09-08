# F1TENTH ROS2 Safety Controller

LiDAR 데이터를 이용해 전방 장애물을 감지하고 F1TENTH 차량의 속도를 제어하는 ROS 2 프로젝트입니다. 차량에 탑재된 ROS 2 Foxy 환경을 기준으로 안전 정지 노드와 조이스틱 데드맨 스위치 기반 주행 노드를 구성했습니다.

## 주요 기능

- `LaserScan`의 전방 ±15° 데이터를 필터링해 가장 가까운 장애물 거리를 계산합니다.
- 장애물 거리에 따라 정지, 저속, 정상 주행의 3단계 속도 명령을 생성합니다.
- 유효한 LiDAR 데이터가 없으면 차량을 정지시키는 fail-safe 동작을 적용했습니다.
- 조이스틱 RB 버튼을 데드맨 스위치로 사용해 버튼을 놓는 즉시 정지 명령을 발행합니다.
- `AckermannDriveStamped` 메시지로 차량 구동 명령을 전달합니다.

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

ROS 2 Foxy와 프로젝트 의존 패키지가 설치된 환경에서 실행합니다.

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
