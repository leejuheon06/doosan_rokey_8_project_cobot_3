# H2017 팔레타이징 아키텍처

이 문서는 현재 실행 코드의 구조만 설명한다. 과거 구현 계획과 완료된 마이그레이션
기록은 유지하지 않는다.

## 프로세스 구성

```text
Isaac Sim (번들 Python 3.11 / ROS 2 Bridge)
  카메라 Action Graph × 2
    ├─ C1: /cv1_depth, /cv_camera1_info, /tf_static
    └─ C2: /cv2_depth, /cv_camera2_info, /tf_static
  PalletizingRosBridge: palletizing_plan_publisher
    ├─ 발행 /conveyor_{1,2}/status, /palletizing/plan
    └─ 구독 /vision/conveyor_{1,2}/box_detections
  application.py
    ├─ IntakeStation × 2
    ├─ DeepPack3D PackingSession × 2
    ├─ RobotUnit × 2
    └─ Conveyor × 2 / Scene / Stability
             │
             │ ROS_DOMAIN_ID=129, rmw_fastrtps_cpp
             ▼
비전 노드 × 2 (시스템 ROS 2 Humble / Python 3.10)
  C1/C2별 depth, camera_info, trigger, /tf 구독
  C1/C2별 box_detections 및 /vision/conveyor_N/debug/* 발행
             │
             ▼
RViz (선택)
  World 고정 프레임에서 영상·점군·3D 박스 표시
```

비전 프로세스는 Open3D, SciPy, OpenCV와 시스템 `rclpy`를 사용하고 Isaac Sim은
번들 Python과 ROS 2 Bridge를 사용하므로 프로세스를 분리한다. 플래너는 NumPy만
쓰는 라이브러리이며 소비자가 시뮬레이터 하나이므로 별도 ROS 노드로 만들지 않는다.

## 온라인 처리 순서

1. 두 `BoxSpawner`가 각 컨베이어에 카탈로그 1~4호 박스를 생성한다.
2. 박스가 해당 라인의 Pick Zone에 도착하면 그 컨베이어만 멈추고 전담 로봇을
   카메라 옆
   측면 대기점으로 보낸다.
3. 해당 `IntakeStation`이 `/conveyor_N/status=True`를 발행해 비전 측정을 요청한다.
4. 비전 노드는 최신 깊이 프레임에서 기준면을 구하고, 기준면 위 점을 군집화한다.
5. 박스 크기 범위 필터로 레일·로봇 링크를 제거하고 카메라 광축에 가장 가까운
   후보 하나를 선택한다.
6. 검출 크기를 공칭 치수로 스냅해 `/vision/conveyor_N/box_detections` JSON으로 보낸다.
7. 해당 로봇의 `PackingSession`이 현재 팔레트 반쪽에 배치 하나를 확정한다.
8. 로봇은 Pick Zone 잠금을 얻어 집는다. yaw 배치는 컨베이어 측면 안전 위치로
   빠져 Pick Zone을 반납한 뒤 목표 자세를 먼저 완성하고, 그 자세를 유지한 채
   지지 박스를 기다렸다가 팔레트에 놓는다.
9. 릴리스 직전 자세와 0.5초 뒤 안정성을 검사한 후 완료로 집계한다.

측정 중 로봇은 상공이 아니라 측면에 머문다. 외부 비전 노드가 트리거 이후의
깊이 프레임을 처리하므로 팔이 카메라와 박스 사이로 먼저 들어가면 박스 점군이
분절되기 때문이다.

## ROS 2 인터페이스

| 토픽 | 타입 | 발행 | 구독 |
|---|---|---|---|
| `/cv1_depth`, `/cv2_depth` | `sensor_msgs/Image` | Isaac 카메라 그래프 | C1/C2 비전 노드 |
| `/cv_camera1_info`, `/cv_camera2_info` | `sensor_msgs/CameraInfo` | Isaac 카메라 그래프 | C1/C2 비전 노드 |
| `/tf_static` | `tf2_msgs/TFMessage` | Isaac 카메라 그래프 | 비전 노드, RViz |
| `/conveyor_1/status`, `/conveyor_2/status` | `std_msgs/Bool` | Isaac 브리지 | C1/C2 비전 노드 |
| `/vision/conveyor_1/box_detections`, `/vision/conveyor_2/box_detections` | `std_msgs/String` JSON | C1/C2 비전 노드 | Isaac 브리지 |
| `/palletizing/plan` | `std_msgs/String` JSON | Isaac 브리지 | 모니터링 도구 |
| `/vision/conveyor_N/debug/*` | 영상·점군·마커 | C1/C2 비전 노드 | RViz |

커스텀 메시지 대신 `std_msgs/String` JSON을 쓰는 이유는 시스템 ROS Humble의
Python 3.10 인터페이스 확장 모듈을 Isaac 번들 Python 3.11에서 직접 import할 수
없기 때문이다.

## 모듈 경계

| 모듈 | 책임 |
|---|---|
| `application.py` | 부팅, 온라인 배정, 두 로봇 병렬 루프, 완료 통계 |
| `catalog.py` | 비전 JSON 파싱과 카탈로그 최근접 스냅 |
| `intake.py` | 측정 트리거, 빈 프레임·null 재시도, 타임아웃 |
| `planning.py` | 격자 배치, 지지율, 온라인 `PackingSession` |
| `robot.py` | 이동 단계, RegionLock, 흡착·릴리스, 제원 계측 |
| `stability.py` | 릴리스 gap/XY/tilt, pose 속도, 정착 후 drift/높이/tilt 판정 |
| `scene.py` | 로컬 자산 교체, AABB, 박스 생성·물리·재질 |
| `conveyor.py` | 컨베이어 속도, 스폰 큐, Pick Zone 도착 판정 |
| `coordination.py` | 유휴 로봇 순위와 공유 구역 잠금 |
| `rmpflow_controller.py` | 로봇별 HOME 기반 RMPflow 설정 생성 |
| `ros_bridge.py` | Isaac 내부의 단일 ROS 노드 |

## 팔레트와 로봇 영역

두 로봇은 `reference3` 셀의 각 컨베이어를 전담한다.

```text
robot_1 base ≈ (1.25, -0.4, 0.5)   conveyor 1
robot_2 base ≈ (1.25,  2.0, 0.5)   conveyor 2
```

살아 있는 팔레트 AABB를 중앙 Y에서 나눠 로봇별 `PackingSession`에 전달한다.
두 세션 모두 중앙선부터 자기 로봇 쪽 외곽으로 채우며 중앙선에는 계획 여백을
두지 않는다. 대신 중앙선 ±0.25 m를 `pallet_center` 교차구역으로 두고, 박스
발자국이 이 구역에 걸리면 두 팔은 먼저 payload 전체가 band에서 0.08 m
밖인 각자의 staging까지 병렬 이동한다. 잠금을 얻은 팔만 place에 진입하며,
수직 retreat 후 staging으로 탈출하는 즉시 `RegionLock`을 반납한다. 측면
대기점으로의 복귀는 다음 팔의 place와 병렬로 진행한다. 중앙 바깥에서는
두 로봇이 계속 병렬로 움직인다. 두 Pick Zone은 서로 다른
`RegionLock`이라 독립 동작하고, C1은 robot_1, C2는 robot_2에 고정 배정된다.
지지 박스가 있는 2단 배치는 해당
박스의 고정 30-step settle 및 안정성 검사가 통과한 뒤에만 시작한다. 릴리스는
계획 지지면 위 6 mm를 목표로 하는 단일 경로만 사용한다. 기본 설정은 안정성
PASS 뒤 박스를 kinematic으로 고정하며 실제 pose occupancy 보정은 사용하지 않는다.

## USD와 카메라

루트 레이어는 `assets/Collected_h2017_gripper/h2017_warehouse_scene.usda`다.
`scene.py`가 팔레트, 컨베이어, RealSense 온라인 참조를 `assets/omni_assets/`
아래 로컬 자산으로 교체한다. 실행에 필요한
카메라는 `/World/conveyor_camera1`, `/World/conveyor_camera2` 두 개이며 팔레트
주변 선택 카메라와 그래프는 비활성화해 렌더 비용을 줄인다.

박스 외관은 크래프트 골판지 기준색 주변에서 별도 난수열로 생성한다. 색 변경이
같은 `--seed`의 박스 호수 순서를 바꾸지 않도록 입고 난수와 색상 난수를 분리했다.

## 완료와 실패 기준

검출 성공, 배치 성공, 물리 안정성 성공은 별도 지표다. 카탈로그 밖 검출이나
팔레트 공간 부족은 해당 박스를 제거하고 통계에 남긴다. 안전하지 않은 릴리스
자세는 오류로 종료한다. 정착 후 XY drift는 5 mm를 넘으면 경고하고 기본
`strict` 정책에서는 25 mm를 넘으면 오류로 종료한다. 지지 높이와 tilt 기준
초과도 즉시 종료한다. 구체적인 현재 수치와 알려진 문제는
[기술 상태](engineering_notes.md)에 정리한다.
