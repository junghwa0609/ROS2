# F1TENTH ROS 2 · LiDAR 거리 기반 속도 제어

LiDAR의 전방 거리를 읽고 정지·감속·주행 명령을 만드는 F1TENTH 프로젝트입니다. 2025년 프로젝트에서는 YDLiDAR G2, Jetson Nano, ROS 2 Foxy, VESC를 사용해 실내 장애물 조건을 비교했습니다. 현재 저장소에는 이후 정리한 계산 함수와 단위 테스트도 포함됩니다.

## 프로젝트에서 다룬 문제

- **센서 수신:** 발행·구독 QoS 조건을 확인하고 센서 데이터 수신 설정을 맞췄습니다.
- **전방 방향:** LiDAR 거리 데이터를 시각화하고 각도와 차량 전방의 관계를 확인했습니다.
- **유효 거리:** 전방 범위 안에서 NaN·무한대·너무 작은 값을 제외하고 가장 가까운 거리를 선택합니다.
- **거리별 판단:** 임계값에 따라 속도를 선택하고 경계조건을 테스트합니다.

프로젝트·논문 작업에 참여했으며 제1저자 기록이 있습니다. 2025 한국정보기술학회 대학생 논문경진대회 우수논문상 동상은 팀 논문 성과입니다.

## 현재 코드의 범위

[control.py](f1tenth_ws/src/f1tenth_system/safety_node/safety_node/control.py)는 각도를 `angle_min + index * angle_increment`로 계산하고, 전방 영역의 유효 거리 중 최소값을 반환합니다. 기본 전방 범위는 ±15도, 최소 유효 거리는 0.05m입니다.

[safety_node.py](f1tenth_ws/src/f1tenth_system/safety_node/safety_node/safety_node.py)는 `/scan`을 센서 데이터 QoS로 구독하고 `/drive`에 `AckermannDriveStamped` 명령을 발행합니다.

| 전방 거리 | 목표 속도 |
| --- | ---: |
| 0.5m 미만 | 0m/s |
| 0.5m 이상 1.5m 미만 | 0.5m/s |
| 1.5m 이상 | 1.5m/s |
| 수신한 스캔에 유효한 전방 거리 없음 | 0m/s |

임계값과 속도는 ROS 2 파라미터로 설정합니다.

**안전 동작의 범위:** 수신한 메시지에 유효값이 없을 때 정지 명령을 만드는 로직입니다. 메시지 수신 자체가 끊겼음을 감지하는 타이머 watchdog은 현재 노드에 없습니다. 물리적인 제동 거리, 통신 단절, 실제 차량의 정지 여부는 별도 시스템 검증이 필요합니다.

## 검증 기록

- 기존 실차 실험: 장애물 없음, 1.2m, 0.3m 조건 비교
- 논문 기록: 100ms 이내 제어 명령 반영. 차량의 물리적 완전 정지 시간과 구분
- 현재 단위 테스트: 전방 필터링, 유효값 없음, 빈 스캔, 정지 조건, 속도 경계, 잘못된 임계값의 6개 테스트
- 이후 정리 이력: [PR #1](https://github.com/junghwa0609/ROS2/pull/1). 최근 코드 정리와 테스트를 과거 실차 실험 결과로 소급하지 않음

## 구조와 시작 위치

```text
f1tenth_ws/src/f1tenth_system/
├── safety_node/       # 거리 계산·속도 정책·노드·테스트
├── f1tenth_stack/     # 차량 bring-up·조이스틱·구동 설정
├── ackermann_mux/    # 명령 우선순위 처리
└── teleop_tools/     # 원격 조작 도구
```

먼저 `safety_node/control.py`와 `tests/test_control.py`를 함께 읽으면 입력값과 판단 조건을 확인할 수 있습니다. 전체 워크스페이스에는 외부 드라이버와 라이브러리가 포함되므로 모든 코드를 개인 작성 코드로 보지 않습니다.

## 빌드·실행

ROS 2 Foxy와 의존 패키지가 설치된 프로젝트 환경을 기준으로 합니다. 하드웨어별 드라이버·포트·토픽·mux 설정은 실제 장비에 맞춰 확인해야 합니다.

```bash
cd f1tenth_ws
colcon build --symlink-install
source install/setup.bash
ros2 run safety_node safety_node.py
```

조이스틱 기반 주행 노드는 다음 실행 경로를 사용합니다. 차량 구성에 맞는 토픽과 데드맨 스위치 설정을 먼저 확인합니다.

```bash
ros2 run f1tenth_stack ttc_driver_with_joy
```

## 계산 함수 테스트

ROS 2 실행 환경 없이 순수 계산 함수 테스트를 실행할 수 있습니다.

```bash
cd f1tenth_ws/src/f1tenth_system/safety_node
python3 -m unittest discover -s tests
```

ROS 2 워크스페이스에서는 다음 경로도 사용할 수 있습니다.

```bash
colcon test --packages-select safety_node
```

이 README 정리 과정에서는 실제 차량·ROS 2 환경을 다시 실행하지 않았습니다.

## 참고·라이선스

이 저장소는 [F1TENTH system](https://github.com/f1tenth/f1tenth_system)을 기반으로 구성했습니다. 원본 드라이버 설치·조이스틱·VESC 설정은 [F1TENTH 문서](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/firmware/index.html)를 참고합니다. 원본 문서의 Hokuyo/urg_node 등 장비 예시와 이 프로젝트의 YDLiDAR 구성을 구분해야 합니다.

포함된 외부 패키지의 라이선스·저작권 표시는 각 디렉터리에 유지되어 있습니다.
