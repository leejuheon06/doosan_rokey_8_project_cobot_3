# H2017 통합 실행 명령 가이드

Isaac Sim, 박스 치수 비전 노드, RViz를 같은 ROS 2 DDS 도메인에서 실행하는
현재 기준 명령 모음이다. 모든 터미널에서 `ROS_DOMAIN_ID`와 RMW 구현이 같아야
서로 토픽을 발견한다.

## 공통 경로와 ROS 설정

새 터미널을 열 때 필요한 값이다. `HOME` 같은 시스템 변수는 덮어쓰지 않는다.

```bash
export SPEEDY_DOUBLE_ROOT="$PWD"  # 저장소 루트에서 실행
export H2017_PROJECT="$SPEEDY_DOUBLE_ROOT/H2017_integrated_V2_speedy_double"
export VISION_PROJECT="$SPEEDY_DOUBLE_ROOT/conveyor_box_measurement_double"
export ISAAC_SIM_PYTHON="<Isaac Sim 설치 디렉터리>/python.sh"
export ISAAC_PY="$ISAAC_SIM_PYTHON"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_logs_h2017_speedy_double
```

ROS CLI, 비전 노드, RViz를 실행하는 터미널에서는 추가로 다음을 적용한다.

```bash
source /opt/ros/humble/setup.bash
```

Isaac Sim 터미널은 `scripts/test5_2robot.py`가 필요한 ROS 2 Bridge 라이브러리를
자동 설정하므로 `/opt/ros/humble/setup.bash`를 source하지 않아도 된다.

## 권장 통합 실행 순서

### 터미널 1: 비전 노드

소스 실행기가 라인별 YAML을 사용해 이름과 토픽이 다른 비전 노드 두 개를 시작한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/ros_logs_h2017_speedy_double

cd "$VISION_PROJECT"
./run_double_vision.sh
```

### 터미널 2: RViz

설정 파일에는 `World` 고정 프레임과 오버레이 영상, ROI 점군, 솟은 점군,
3D 박스 마커가 미리 등록돼 있다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

rviz2 -d "$H2017_PROJECT/config/vision_debug.rviz"
```

### 터미널 3: Isaac Sim

빠른 정상 확인에는 라인별 1개씩 투입되는 검증 완료 2개 박스 조건을 권장한다.

```bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export H2017_PROJECT="$SPEEDY_DOUBLE_ROOT/H2017_integrated_V2_speedy_double"
export ISAAC_PY="$ISAAC_SIM_PYTHON"

cd "$H2017_PROJECT"
"$ISAAC_PY" scripts/test5_2robot.py --box-count 2 --seed 42
```

2개 조건의 최근 검증 결과는 각 라인 1개, 비전 검출·분류 `2/2`, 적재 안정성
`2/2 PASS`다. 더 긴 적재 검증은 양 라인에 동일한 수가 배분되도록 짝수 개수를 쓴다.

```bash
cd "$H2017_PROJECT"
"$ISAAC_PY" scripts/test5_2robot.py --box-count 6 --seed 42
```

## 자주 쓰는 시뮬레이션 옵션

| 옵션 | 기본값 | 설명 |
|---|---:|---|
| `--box-count N` | 25 | 투입할 박스 수 |
| `--box-numbers RANGE` | `1-4` | 스폰할 호수. `2-4` 또는 `1,3,4` 형식 |
| `--seed N` | 매번 랜덤 | 재현 가능한 박스 순서와 색상 |
| `--spawn-interval SEC` | 5.0 | 박스 생성 최소 간격(물리 시간) |
| `--measurement-settle-steps N` | 0 | Pick Zone 정지 후 비전 트리거 전 대기 step |
| `--packing-method` | `baf` | `bl`, `baf`, `bssf`, `blsf` |
| `--stability-policy` | `strict` | `strict` / drift-only 실패를 계속하는 `continue-drift` |
| `--freeze-after-stability` | 켜짐 | 안정성 PASS 뒤 박스를 kinematic으로 고정 |
| `--placer` | `default` | `default`(원본) / `stable`(안정성 필터 추가) |
| `--stable-min-horizontal-support` | 0.3 | `stable`에서 요구하는 최소 측면 지지 비율 |
| `--stable-min-com-margin` | 0.05 | `stable`에서 요구하는 최소 무게중심 여유 비율 |
| `--yaw-rotation` | 꺼짐 | 두 로봇 모두 계획 yaw를 측면 staging에서 먼저 맞춘 뒤 place까지 고정 |
| `--headless` | 꺼짐 | GUI 없이 실행 |

```bash
# 2~4호 박스만 무작위로 스폰
"$ISAAC_PY" scripts/test5_2robot.py --box-count 10 --box-numbers 2-4 --seed 42
```

```bash
# 빠른 GUI 검증
"$ISAAC_PY" scripts/test5_2robot.py --box-count 5 --seed 42

# 화면 없는 반복 검증
"$ISAAC_PY" scripts/test5_2robot.py --box-count 5 --seed 42 --headless

# 다른 스폰 간격
"$ISAAC_PY" scripts/test5_2robot.py --box-count 10 --spawn-interval 7 --seed 13

# 패킹 휴리스틱 비교
"$ISAAC_PY" scripts/test5_2robot.py --box-count 10 --seed 13 --packing-method bssf

# 안정성 필터 켜기 (--seed를 고정하면 default와 같은 박스 세트로 비교된다)
"$ISAAC_PY" scripts/test5_2robot.py --box-count 10 --seed 13 --placer stable

# 두 로봇에서 90도 yaw 배치 실행
"$ISAAC_PY" scripts/test5_2robot.py --box-count 40 --seed 13 --placer stable \
  --packing-method bssf --yaw-rotation
```

### 적재 안정성 정책

릴리스 뒤 30 physics step에서 한 번 판정한다. 기본 `strict`는 XY drift 25 mm,
지지 높이 오차 5 mm, tilt 2° 중 하나라도 넘으면 오류로 종료한다. 5~25 mm의
XY drift는 WARNING으로 기록하고 계속한다. 기본값으로 PASS 박스는 고정하지만
실제 정착 pose를 패커 occupancy에 반영하지 않는다.

### `--placer stable`이 하는 일

기본 플래너는 "발자국의 절반 넘게 받쳐지는가"만 본다. 지지가 한쪽으로 몰려
무게중심이 지지영역 밖으로 나가는 배치도, 옆에서 아무것도 받쳐주지 않는
배치도 통과한다 — 격자 위에서는 합법이지만 실물은 넘어진다.

`stable`은 두 가지를 추가로 본다:

- **측면 지지** (`--stable-min-horizontal-support`): 박스의 -x/-y 옆면이
  팔레트 벽이나 이웃 박스와 맞닿는 면적 비율. 교차적재가 되어 있는가
- **무게중심 여유** (`--stable-min-com-margin`): 박스 중심에서 지지영역
  경계까지의 거리를 지지영역의 짧은 변으로 나눈 값. 0.5면 한가운데,
  0.0이면 경계에 정확히 걸린 상태다

기본값(0.3 / 0.05)은 Isaac 물리로 실측 검증된 설정이다 — 붕괴 런이 20회 중
6회에서 1회로 준다. 자세한 근거는 `docs/packing_algorithms.md` §4 참조.

## 주요 토픽

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/cv1_depth`, `/cv2_depth` | `sensor_msgs/Image` | 라인별 깊이 영상 |
| `/cv_camera1_info`, `/cv_camera2_info` | `sensor_msgs/CameraInfo` | 라인별 카메라 내부 파라미터 |
| `/conveyor_1/status`, `/conveyor_2/status` | `std_msgs/Bool` | 라인별 비전 측정 트리거 |
| `/vision/conveyor_1/box_detections`, `/vision/conveyor_2/box_detections` | `std_msgs/String` | 라인별 박스 중심·크기 JSON |
| `/vision/conveyor_N/debug/overlay_image` | `sensor_msgs/Image` | 라인별 RViz 검출 오버레이 |
| `/vision/conveyor_N/debug/pointcloud` | `sensor_msgs/PointCloud2` | 라인별 ROI 전체 점군 |
| `/vision/conveyor_N/debug/raised_points` | `sensor_msgs/PointCloud2` | 라인별 기준면 위 점군 |
| `/vision/conveyor_N/debug/markers` | `visualization_msgs/MarkerArray` | 라인별 박스와 치수 마커 |
| `/palletizing/plan` | `std_msgs/String` | 누적 팔레타이징 계획 JSON |

## 상태 점검

다음 명령은 ROS 설정을 적용한 별도 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 node list --no-daemon
ros2 topic list --no-daemon | sort
ros2 topic info /cv1_depth --verbose --no-daemon
ros2 topic info /cv2_depth --verbose --no-daemon
ros2 topic echo /vision/conveyor_1/box_detections --once --no-daemon
ros2 topic echo /vision/conveyor_2/box_detections --once --no-daemon
```

발행 주파수 확인:

```bash
timeout 8s ros2 topic hz /cv1_depth
timeout 8s ros2 topic hz /cv2_depth
```

정상 연결이면 RViz 토픽의 Publisher와 Subscription count가 각각 1 이상이다.

## 종료와 잔류 프로세스 확인

각 실행 터미널에서 `Ctrl+C`로 종료하는 것이 가장 안전하다. 종료 후 확인한다.

```bash
pgrep -af 'test5_2robot.py|conveyor_box_measurement.node|rviz2|isaacsim.exp.base.python.kit'
```

터미널을 잃어버려 프로세스가 남은 경우에만 개별적으로 SIGINT를 보낸다.

```bash
pkill -INT -f 'scripts/test5_2robot.py'
pkill -INT -f 'conveyor_box_measurement.node'
pkill -INT -x rviz2
```

Isaac Kit 자식만 남았는지는 다시 `pgrep`로 확인한다. 광범위한
`pkill python3`나 `killall python3`는 다른 ROS/Python 작업까지 종료하므로 쓰지 않는다.

## 테스트

Isaac Sim이 필요 없는 프로젝트 단위 테스트:

```bash
cd "$H2017_PROJECT"
env PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
```

비전 패키지 테스트:

```bash
source /opt/ros/humble/setup.bash
cd "$SPEEDY_DOUBLE_ROOT"
env PYTHONPATH=conveyor_box_measurement_double/src \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -p no:anyio \
  conveyor_box_measurement_double/test
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`은 시스템 pytest와 사용자 영역 플러그인의 버전
충돌을 피하기 위한 현재 환경 설정이다.

## 자주 보는 출력 파일과 설정

```text
outputs/palletizing_plan.json          누적 적재 계획
outputs/rmpflow/                       실행 시 생성되는 로봇 서술 파일
config/vision_debug.rviz               RViz 통합 표시 설정
../conveyor_box_measurement_double/config/measurement_conveyor_1.yaml
../conveyor_box_measurement_double/config/measurement_conveyor_2.yaml
                                       라인별 비전 토픽·ROI·클러스터 설정
```
