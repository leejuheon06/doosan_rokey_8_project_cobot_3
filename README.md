# Speedy Double 실행 가이드

Isaac Sim 기반 2대 로봇 팔레타이징 시뮬레이션과 ROS 2 비전 노드, 웹 모니터링
UI를 한 워크스페이스에서 실행하는 프로젝트다.

## 구성

| 구성 요소 | 역할 | 실행 명령 |
|---|---|---|
| Launcher | 웹 UI 명령을 받아 Isaac Sim을 시작·종료 | `ros2 run system_monitor_ui launcher` |
| UI Server | ROS 데이터를 웹 화면으로 제공 | `ros2 run system_monitor_ui server` |
| Vision Node | C1/C2 Depth로 박스와 Point Cloud 측정 | `./run_double_vision.sh` |

세 프로세스는 모두 같은 `ROS_DOMAIN_ID`와 RMW 구현을 사용해야 한다. 이 프로젝트의
기본값은 `ROS_DOMAIN_ID=129`, `rmw_fastrtps_cpp`다.

이 디렉터리(`cobot3_ws/src`)에는 이 워크플로우에서 쓰는 세 개의 패키지/프로젝트가
있다.

| 패키지 | 빌드 타입 | 역할 |
|---|---|---|
| `system_monitor_ui` | `ament_python` | Launcher/Server 노드와 웹 대시보드(Flask) |
| `conveyor_box_measurement_double` | `ament_python`, colcon 패키지명 `conveyor_box_measurement` | C1/C2 Depth 비전 측정 노드, `run_double_vision.sh` |
| `H2017_integrated_V2_speedy_double` | ROS 패키지 아님(`package.xml` 없음) | Isaac Sim 시뮬레이션 프로젝트. `scripts/test5_2robot.py`를 Isaac Sim Python으로 직접 실행 |

## 요구 환경

- Ubuntu 및 ROS 2 Humble
- Isaac Sim Python 실행 파일(`ISAAC_SIM_PYTHON` 환경변수로 지정)
- Python 패키지: Flask, Pillow, NumPy, OpenCV, Open3D
- ROS 패키지: `rclpy`, `sensor_msgs`, `visualization_msgs`, `cv_bridge`, `tf2_ros`

Isaac Sim 설치 위치는 PC마다 다르므로 실행 전에 `python.sh`(또는 배포판에 따라
`python.sh`/`isaacsim-python`) 경로를 확인해 `ISAAC_SIM_PYTHON`으로 지정한다.
경로를 모를 때는 다음으로 후보를 찾을 수 있다.

```bash
find ~ -maxdepth 6 -iname "python.sh" 2>/dev/null
```

## 워크스페이스 구조

이 프로젝트는 표준 colcon 워크스페이스인 `~/cobot3_ws` 안에 있다.

```text
~/cobot3_ws/
├── build/, install/, log/     colcon 빌드 산출물(워크스페이스 루트에 생성됨)
└── src/                       ← 이 README가 있는 디렉터리. 패키지들이 직접 위치
    ├── system_monitor_ui/
    ├── conveyor_box_measurement_double/
    └── H2017_integrated_V2_speedy_double/
```

아래 명령에서는 다음 두 변수를 사용한다.

- `COBOT3_WS`: 워크스페이스 루트(`build`/`install`/`log`가 있는 곳)
- `SPEEDY_DOUBLE_ROOT`: 패키지들이 직접 위치한 `src` 디렉터리. Launcher가
  `H2017_integrated_V2_speedy_double/scripts/test5_2robot.py`를 찾는 기준
  경로이자, 하위 패키지 README들이 말하는 "저장소 루트"다.

다른 위치에 클론했다면 `~/cobot3_ws`만 실제 경로로 바꾸면 된다.

## 최초 빌드

```bash
export COBOT3_WS="$HOME/cobot3_ws"
export SPEEDY_DOUBLE_ROOT="$COBOT3_WS/src"
cd "$COBOT3_WS"
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src/system_monitor_ui src/conveyor_box_measurement_double \
  --ignore-src -r -y

colcon build --symlink-install \
  --packages-select system_monitor_ui conveyor_box_measurement
```

`rosdep`에서 `python3-open3d`를 설치하지 못하는 경우에만 다음 명령을 사용한다.

```bash
python3 -m pip install open3d
```

소스나 설정을 수정한 뒤에는 위의 `colcon build` 명령을 다시 실행한다.
`--symlink-install` 빌드이므로 Python 및 웹 템플릿 변경은 대부분 즉시 반영되지만,
실행 중인 프로세스는 재시작해야 한다.

## 실행 방법

아래 명령은 터미널 3개에서 각각 실행한다. 실행 순서는 Launcher → UI Server →
Vision Node를 권장한다. Vision Node는 Isaac Sim의 Depth 토픽이 나오기 전부터 켜
두어도 된다.

### 터미널 1 — Launcher

```bash
export COBOT3_WS="$HOME/cobot3_ws"
export SPEEDY_DOUBLE_ROOT="$COBOT3_WS/src"
source /opt/ros/humble/setup.bash
source "$COBOT3_WS/install/setup.bash"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ISAAC_SIM_PYTHON="<Isaac Sim 설치 디렉터리>/python.sh"
export SPEEDY_DOUBLE_PROJECT="$SPEEDY_DOUBLE_ROOT/H2017_integrated_V2_speedy_double"

ros2 run system_monitor_ui launcher
```

Launcher는 `SPEEDY_DOUBLE_PROJECT`가 없으면 현재 작업 디렉터리를 기준으로
`H2017_integrated_V2_speedy_double/scripts/test5_2robot.py`를 위로 탐색한다.
이 워크스페이스는 패키지가 `cobot3_ws/src` 아래에 있어 워크스페이스 루트
(`cobot3_ws`)에서 실행하면 자동 탐색에 실패하므로, 위와 같이
`SPEEDY_DOUBLE_PROJECT`를 명시하거나 `cd "$SPEEDY_DOUBLE_ROOT"` 후 실행한다.

Launcher는 직접 Isaac Sim을 실행하지 않고 대기한다. 웹 UI에서 **시뮬레이션 시작**을
누르면 다음 파일을 Isaac Sim Python으로 실행한다.

```text
H2017_integrated_V2_speedy_double/scripts/test5_2robot.py
```

### 터미널 2 — 웹 UI Server

```bash
export COBOT3_WS="$HOME/cobot3_ws"
source /opt/ros/humble/setup.bash
source "$COBOT3_WS/install/setup.bash"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run system_monitor_ui server
```

브라우저에서 다음 주소를 연다.

```text
http://localhost:5000/smu
```

다른 컴퓨터에서 접속할 때는 `localhost` 대신 UI 서버 컴퓨터의 IP를 사용한다.
화면에서 실행 조건을 설정하고 **시뮬레이션 시작**을 누르면 Isaac Sim이 실행된다.

### 터미널 3 — C1/C2 비전 노드

```bash
export COBOT3_WS="$HOME/cobot3_ws"
export SPEEDY_DOUBLE_ROOT="$COBOT3_WS/src"
source /opt/ros/humble/setup.bash
source "$COBOT3_WS/install/setup.bash"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd "$SPEEDY_DOUBLE_ROOT/conveyor_box_measurement_double"
./run_double_vision.sh
```

이 스크립트는 다음 두 노드를 동시에 실행한다.

- `conveyor_1_measurement`: `/cv1_depth`, `/cv_camera1_info` 사용
- `conveyor_2_measurement`: `/cv2_depth`, `/cv_camera2_info` 사용

노드를 별도 터미널에서 수동 실행하지 않는다. `run_double_vision.sh`를 두 번 실행하면
라인별 Publisher가 중복되어 검출 결과와 디버그 데이터도 중복 발행될 수 있다.

### Launcher/UI 없이 Isaac Sim만 직접 실행하기

디버깅 등으로 웹 UI를 거치지 않고 시뮬레이터만 띄우려면 다음과 같이 실행한다.
옵션 전체 목록은
[`H2017_integrated_V2_speedy_double/docs/test5_run_guide.md`](H2017_integrated_V2_speedy_double/docs/test5_run_guide.md)에
있다.

```bash
export COBOT3_WS="$HOME/cobot3_ws"
export SPEEDY_DOUBLE_ROOT="$COBOT3_WS/src"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ISAAC_SIM_PYTHON="<Isaac Sim 설치 디렉터리>/python.sh"

cd "$SPEEDY_DOUBLE_ROOT/H2017_integrated_V2_speedy_double"
"$ISAAC_SIM_PYTHON" scripts/test5_2robot.py --box-count 2 --seed 42
```

## 정상 동작 확인

Isaac Sim 창이 열린 뒤 타임라인이 재생 중이어야 Depth가 발행된다. PhysX PBD는
Depth/ROS Bridge/Point Cloud 수신 조건이 아니다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/cobot3_ws/install/setup.bash"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 node list
ros2 topic hz /cv1_depth
ros2 topic hz /cv2_depth
ros2 topic hz /vision/conveyor_1/debug/pointcloud
ros2 topic hz /vision/conveyor_2/debug/pointcloud
```

UI 서버 확인:

```bash
curl http://127.0.0.1:5000/api/state
```

웹 UI의 **DEPTH / 3D POINTS** 버튼으로 영상을 전환한다. 3D 화면의 표시는 다음과
같다.

- 녹색 격자: `CONVEYOR TOP · 0 mm`
- 청록색: 측정 ROI Point Cloud
- 주황색: 컨베이어 기준면 위의 관측점
- 노란색 외곽선: 검출된 박스
- `SIDE / TOP / ISO`: 기준면 시점 전환

박스가 측정 ROI에 없으면 주황색 포인트와 노란색 외곽선이 표시되지 않는 것이
정상이다.

## RViz에서 확인

```bash
export SPEEDY_DOUBLE_ROOT="$HOME/cobot3_ws/src"
source /opt/ros/humble/setup.bash
source "$HOME/cobot3_ws/install/setup.bash"
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

rviz2 -d "$SPEEDY_DOUBLE_ROOT/H2017_integrated_V2_speedy_double/config/vision_debug.rviz"
```

RViz의 Fixed Frame은 `World`를 사용한다. 비전 디버그 발행 설정은 다음 파일에서
수정할 수 있다.

- `conveyor_box_measurement_double/config/measurement_conveyor_1.yaml`
- `conveyor_box_measurement_double/config/measurement_conveyor_2.yaml`

## 종료

각 터미널에서 `Ctrl+C`로 종료한다. 권장 순서는 Vision Node → UI Server →
Launcher다. 시뮬레이션만 멈추려면 웹 UI의 **중지** 버튼을 사용한다. Launcher를
`Ctrl+C`로 종료하면 Launcher가 시작한 Isaac Sim에도 `SIGINT`를 전달한다.

실행 중인 관련 프로세스 확인:

```bash
ps -eo pid,ppid,etime,stat,cmd | \
  grep -E 'system_monitor_ui|conveyor_box_measurement|test5_2robot.py|isaacsim' | \
  grep -v grep
```

## 문제 해결

### UI에 `Launcher Offline`이 표시됨

Launcher와 Server 터미널의 `ROS_DOMAIN_ID` 및 `RMW_IMPLEMENTATION`이 같은지
확인하고, 두 터미널 모두 `$COBOT3_WS/install/setup.bash`를 source했는지 확인한다.

### Launcher가 `Cannot find H2017_integrated_V2_speedy_double`로 종료됨

Launcher를 실행한 작업 디렉터리 기준으로 프로젝트 폴더를 찾지 못한 것이다.
`SPEEDY_DOUBLE_PROJECT`를 `$COBOT3_WS/src/H2017_integrated_V2_speedy_double`로
지정했는지 확인한다.

### DEPTH 또는 3D POINTS가 들어오지 않음

1. Isaac Sim 타임라인이 재생 중인지 확인한다.
2. `/cv1_depth`, `/cv2_depth`의 주기를 확인한다.
3. 비전 노드가 라인별로 정확히 하나씩 실행 중인지 확인한다.
4. Isaac Sim의 ROS 2 Bridge, Render Product, Camera Helper/Action Graph를 확인한다.

### 포트 5000이 이미 사용 중임

```bash
ss -ltnp | grep ':5000'
```

기존 UI 서버를 종료한 뒤 다시 실행한다.

## 세부 문서

- 시뮬레이션: `H2017_integrated_V2_speedy_double/README.md`,
  통합 실행 명령 전체는 `H2017_integrated_V2_speedy_double/docs/test5_run_guide.md`
- 비전 노드: `conveyor_box_measurement_double/README.md`
- 웹 UI: `system_monitor_ui/README.md`

하위 README들은 "저장소 루트"를 이 디렉터리(`cobot3_ws/src`)로 간주하고
`SPEEDY_DOUBLE_ROOT`를 사용하므로, 여기서 정의한 `SPEEDY_DOUBLE_ROOT` 값을
그대로 사용하면 된다.
