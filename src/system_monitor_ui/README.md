# Speedy Double System Monitor UI

Speedy Double의 실행 설정, 프로세스 상태, 실시간 로그와 종료 지표를 제공하는
ROS 2 기반 웹 UI다. 브라우저와 Flask는 HTTP로 통신하고, Flask와 실행 Launcher는
ROS 2 토픽으로 통신한다.

## ROS 2 토픽

| 토픽 | 방향 | 형식 |
|---|---|---|
| `/palletizing/control` | UI → Launcher | `std_msgs/String` JSON |
| `/palletizing/process_state` | Launcher → UI | `std_msgs/String` JSON |
| `/palletizing/log` | Launcher → UI | `std_msgs/String` JSON |
| `/palletizing/result` | Launcher → UI | `std_msgs/String` JSON |
| `/vision/conveyor_1/box_detections` | Vision C1 → UI | `std_msgs/String` JSON |
| `/vision/conveyor_2/box_detections` | Vision C2 → UI | `std_msgs/String` JSON |
| `/vision/conveyor_1/debug/overlay_image` | Vision C1 → UI | `sensor_msgs/Image` (`bgr8`) |
| `/vision/conveyor_2/debug/overlay_image` | Vision C2 → UI | `sensor_msgs/Image` (`bgr8`) |
| `/vision/conveyor_1/debug/pointcloud` | Vision C1 → UI | `sensor_msgs/PointCloud2` XYZ |
| `/vision/conveyor_2/debug/pointcloud` | Vision C2 → UI | `sensor_msgs/PointCloud2` XYZ |
| `/vision/conveyor_1/debug/raised_points` | Vision C1 → UI | `sensor_msgs/PointCloud2` XYZ |
| `/vision/conveyor_2/debug/raised_points` | Vision C2 → UI | `sensor_msgs/PointCloud2` XYZ |

## 빌드

```bash
# 압축을 푼 저장소 루트에서 실행
export SPEEDY_DOUBLE_ROOT="$PWD"
colcon build --symlink-install --packages-select system_monitor_ui
source "$SPEEDY_DOUBLE_ROOT/install/setup.bash"
```

## 실행

두 터미널을 저장소 루트에서 열고 같은 ROS 설정을 사용한다. 각 터미널에서 먼저
`export SPEEDY_DOUBLE_ROOT="$PWD"`를 실행한다.

터미널 1 — Isaac Sim Launcher:

```bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source "$SPEEDY_DOUBLE_ROOT/install/setup.bash"
ros2 run system_monitor_ui launcher
```

터미널 2 — 웹 UI:

```bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source "$SPEEDY_DOUBLE_ROOT/install/setup.bash"
ros2 run system_monitor_ui server
```

브라우저에서 `http://localhost:5000/smu`를 연다. Launcher가 연결된 뒤 설정을
선택하고 **시뮬레이션 시작**을 누르면 Speedy Double이 실행된다.

비전 노드를 함께 실행하면 프로세스 상태 아래의 **Depth Vision 실시간 측정**에
C1/C2 컬러 Depth, 측정 ROI, 검출 외곽선과 `L × W × H` 치수가 표시된다. 웹
서버는 ROS `bgr8` 오버레이를 JPEG로 변환해 `/api/vision/N/frame.jpg`로 제공한다.
최근 2.5초 동안 영상이 없으면 해당 카드는 `DEPTH OFFLINE`으로 전환된다.

각 카드의 **DEPTH / 3D POINTS** 버튼으로 2D 컬러 Depth와 인터랙티브 점군을
전환할 수 있다. 3D 화면은 ROI 전체 점을 청록색, 기준면보다 솟은 점을 주황색으로
표시한다. 녹색 격자와 `CONVEYOR TOP · 0 mm`는 측정된 컨베이어 상단 기준면이고,
`+HEIGHT`는 이 면에서 위쪽인 방향, `BELT` 화살표는 컨베이어 진행 방향이다.
**SIDE / TOP / ISO** 버튼으로 기준면의 옆·위·입체 시점을 즉시 확인할 수 있으며,
드래그 회전, 휠 확대, 더블클릭 시점 초기화도 지원한다. 기준면 아래와 평면 영역
밖의 배경 점은 숨기고, 검출된 박스는 노란색 3D 외곽선으로 겹쳐 표시한다. 서버는
브라우저 부하를 제한하기 위해 PointCloud2를 다운샘플하고 little-endian XYZ
바이너리로 `/api/vision/N/cloud.bin?kind=roi|raised`에서 제공한다.

Isaac Sim의 PhysX PBD 설정은 이 경로와 무관하다. 카메라 Render Product,
ROS 2 Camera Helper/Action Graph, `isaacsim.ros2.bridge`, 재생 중인 타임라인이
Depth 발행 조건이며, 웹용 Point Cloud는 비전 노드가 `/cvN_depth`에서 생성한다.

## 종료 지표

정상 완료 시 투입·적재·제거 수, 시스템 takt, 처리량, 로봇별 cycle time,
Pick Zone 지연, 비전 검출·분류 정확도, 안정성 PASS/WARNING과 drift, 공간효율을
표시하고 SQLite 실행 기록에 저장한다. 공간효율은 바닥 점유율(`F`)과 수직
압축률(`C`)의 조화평균에 안정성 통과율(`S`)을 곱한다.

```text
공간효율 = (2 * F * C / (F + C)) * S * 100
```

화면에는 이 프로젝트 종합 KPI만 대표 공간효율로 표시한다. 원래 물리
체적효율과 세부 구성 값은 결과 JSON에 분석용으로 계속 저장한다.
