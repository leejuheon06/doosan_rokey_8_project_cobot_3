# conveyor_box_measurement_double

H2017 팔레타이징 시뮬레이션의 깊이 카메라 영상에서 박스 중심, 크기, yaw를
측정하는 독립 ROS 2 Humble 패키지입니다. 시뮬레이션과 다른 컴퓨터에서 실행할
수 있으며 양쪽 컴퓨터가 같은 DDS 네트워크에 있으면 됩니다.

이 복사본은 두 카메라를 서로 다른 노드와 토픽으로 처리한다. 기본 연결 대상:

- 시뮬레이션: 저장소의 `H2017_integrated_V2_speedy_double/`
- C1 입력: `/cv1_depth`, `/cv_camera1_info`, `/conveyor_1/status`, `/tf`
- C2 입력: `/cv2_depth`, `/cv_camera2_info`, `/conveyor_2/status`, `/tf`
- C1 출력: `/vision/conveyor_1/box_detections` (`std_msgs/msg/String`, JSON)
- C2 출력: `/vision/conveyor_2/box_detections` (`std_msgs/msg/String`, JSON)
- 출력 좌표계: `World`
- RMW: `rmw_fastrtps_cpp`

## 설치

비전 컴퓨터의 ROS 2 워크스페이스에 이 디렉터리를 복사합니다.

```bash
mkdir -p ~/vision_ws/src
cp -a conveyor_box_measurement_double ~/vision_ws/src/
source /opt/ros/humble/setup.bash
cd ~/vision_ws
rosdep install --from-paths src --ignore-src -r -y
```

```bash
colcon build --symlink-install --packages-select conveyor_box_measurement
```

`rosdep`가 배포판에서 `python3-open3d`를 설치하지 못하는 경우에만 시스템
Python에 `python3 -m pip install open3d`로 설치합니다.

## 실행

시뮬레이션과 비전 컴퓨터에서 `ROS_DOMAIN_ID`와 RMW 구현을 동일하게 둡니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=129
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SPEEDY_DOUBLE_ROOT="$PWD"  # 저장소 루트에서 실행
cd "$SPEEDY_DOUBLE_ROOT/conveyor_box_measurement_double"
./run_double_vision.sh
```

다른 네트워크 인터페이스나 서브넷을 사용한다면 Fast DDS discovery와 방화벽도
별도로 설정해야 합니다. C1/C2 토픽과 ROI 값은 각각
`config/measurement_conveyor_1.yaml`, `config/measurement_conveyor_2.yaml`에서
바꿀 수 있습니다. `run_double_vision.sh`는 두 파일을 명시해 노드 이름까지 분리한다.
단일 노드로 사용할 때는 기존 `config/measurement.yaml`이 기본값이다.

`empty_detection_retry_frames`의 기본값은 2입니다. 측정 트리거 직후 받은
Depth 프레임에서 박스가 보이지 않으면 `box: null`을 즉시 발행하지 않고 새
프레임을 최대 2장 더 확인합니다. 카메라 갱신 지연이 있어도 최초 프레임을 포함해
최대 3프레임 안에서 검출할 수 있습니다.

출력 예시는 다음과 같습니다.

```json
{"stamp": 1786174308, "frame_id": "World", "box": {"id": 3, "center_m": [0.23, -0.71, 0.78], "size_m": [0.20, 0.15, 0.15], "yaw_rad": -1.57}}
```

## RViz 실시간 확인

기본 설정은 5 Hz로 디버그 시각화를 발행합니다. RViz의 Fixed Frame을 `World`로
설정하고 다음 Display를 추가합니다.

| Display | Topic | 내용 |
|---|---|---|
| Image | `/vision/conveyor_N/debug/overlay_image` | 컬러 depth, ROI, 검출 박스와 치수 |
| Image | `/vision/conveyor_N/debug/depth_image` | 미터 단위 `32FC1` 원본 depth |
| PointCloud2 | `/vision/conveyor_N/debug/pointcloud` | 핀홀 모델로 역투영한 ROI 점군 |
| PointCloud2 | `/vision/conveyor_N/debug/raised_points` | 컨베이어 기준면보다 솟은 점 |
| MarkerArray | `/vision/conveyor_N/debug/markers` | 기준면, 3D 박스, 중심점과 치수 텍스트 |

점군이 너무 무거우면 `debug_point_stride`를 키우거나 `debug_publish_rate_hz`를
낮춥니다. 운영할 때 시각화가 필요 없으면 `publish_debug: false`로 끕니다.
