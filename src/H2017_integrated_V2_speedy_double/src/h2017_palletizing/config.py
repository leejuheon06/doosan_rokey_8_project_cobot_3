
"""Runtime configuration for the two-robot H2017 palletizer."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = PROJECT_ROOT / "assets"
RMPFLOW_DIR = PROJECT_ROOT / "config" / "rmpflow"

# ╚══════════════════════════════════════════════════════════════╝
# 창고 배경(my_warehouse_base.usd)을 약한 레이어로, 2라인 로봇 셀
# (h2017_gripper_v8.usd)을 강한 레이어로 합성한다.
# 창고와 2라인 셀을 합성한 실행 루트다.
USD_PATH = str(
    ASSETS_DIR / "Collected_h2017_gripper" / "h2017_warehouse_scene.usda"
)
# USD_PATH = str(ASSETS_DIR / "Collected_h2017_gripper" / "h2017_gripper_v8.usd")
LOCAL_PALLET_USD = str(
    ASSETS_DIR
    / "omni_assets/Assets/Isaac/v5_1/Isaac/Props/Pallet/pallet.usd"
)
LOCAL_CONVEYOR_USD = str(
    ASSETS_DIR
    / "omni_assets/Assets/Isaac/v5_1/Isaac/Props/Conveyors/ConveyorBelt_A06.usd"
)
LOCAL_RSD455_USD = str(
    ASSETS_DIR
    / "omni_assets/Assets/Isaac/v5_1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
CONVEYOR_1_PRIM_PATHS = (
    "/World/ConveyorTrack",
    "/World/ConveyorTrack_01",
    "/World/ConveyorTrack_02",
)
CONVEYOR_2_PRIM_PATHS = (
    "/World/ConveyorTrack_04",
    "/World/ConveyorTrack_05",
    "/World/ConveyorTrack_06",
)
# 두 컨베이어 카메라는 라인별 비전 노드가 박스 치수를 재는 입력이다. 온라인
# payload를 번들된 로컬 자산으로 교체해 살려 두며, 액션 그래프는 C1/C2별 depth와
# camera_info를 발행한다.
LOCAL_CAMERA_PRIM_PATHS = (
    "/World/conveyor_camera1",
    "/World/conveyor_camera2",
)
# 나머지 카메라는 이 런타임에서 쓰지 않는다. 렌더 비용만 들므로 비활성화한다.
OPTIONAL_CAMERA_PRIM_PATHS = (
    "/World/pallet_camera",
    "/World/rsd455_x1",
    "/World/rsd455_x2",
    "/World/rsd455_y1",
    "/World/rsd455_y2",
)
OPTIONAL_CAMERA_GRAPH_PATHS = ("/World/Graph/pt_camera_graph",)

# usd 안에 이미 배치되어 있는 큐브/pallet 프림 경로.
CONVEYOR_TEMPLATE_CUBE_PATHS = (
    "/World/Cube",
    "/World/Cube_01",
)
SPAWNED_CUBE_ROOT_PATH = "/World/AutoSpawnedCubes"
# 생성/적재할 박스 수. --box-count로 덮어쓸 수 있다.
# 팔레트를 실제로 채우려면 이 정도가 필요하다. 실측(측정 스크립트, 8개 시드)에서
# 25개는 전부 배치 성공, 1층 바닥면적 35%에 평균 2.3층으로 쌓였다. 40개를 넘기면
# 배치 실패가 나기 시작한다.
DEFAULT_BOX_COUNT = 25
PALLET_PRIM_PATH = "/World/pallet"
# 팔레트의 X/Y 위치. Z는 모델 원점이 아니라 실제 AABB 바닥면이 FLOOR_Z에
# 닿도록 로드 시 자동 계산한다.
PALLET_WORLD_XY = np.array([1.2, 0.8], dtype=np.float64)

EE_FRAME_NAME = "link_6"

# 장면에는 각자 전용 컨베이어를 바라보는 H2017이 두 대 있다.
#   robot_1: base (1.25, -0.4, 0.5)  — conveyor 1
#   robot_2: base (1.25,  2.0, 0.5)  — conveyor 2
# 두 대가 같은 pick/place 알고리즘을 그대로 공유하고, 다른 것은 베이스 위치와
# HOME 자세뿐이다. 각자 컨베이어 쪽을 봐야 하므로 1축이 서로 π 만큼 반대다.
# v3는 실제 로봇(h2017_gripper_v2.usd)을 payload로 감싼 래퍼라서
# 기존 경로 앞에 "h2017_gripper_v2"가 한 단계 더 붙는다.
ROBOT_CONFIGS = (
    {
        "name": "robot_1",
        "prefix": "/World/h2017_gripper_v2_01",
        "home_pose": np.array([np.pi / 2, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0]),
    },
    {
        "name": "robot_2",
        "prefix": "/World/h2017_gripper_v2",
        "home_pose": np.array([-np.pi / 2, 0.0, np.pi / 2.0, 0.0, np.pi / 2.0, 0.0]),
    },
)

DRIVE_MIN_STIFFNESS = 7000.0
DRIVE_MAX_STIFFNESS = 15000.0
DRIVE_MIN_DAMPING = 20.0
DRIVE_MAX_DAMPING = 500.0
VG10_MASS_KG = 1.62

H2017_URDF_PATH = str(ASSETS_DIR / "doosan-robot2" / "urdf" / "h2017.urdf")
H2017_DESCRIPTION_PATH = str(RMPFLOW_DIR / "h2017_description.yaml")
H2017_RMPFLOW_CONFIG_PATH = str(RMPFLOW_DIR / "h2017_rmpflow_common.yaml")
# h2017_description.yaml의 default_q는 단일 로봇 기준(joint_1=0)이라 이 장면의
# 두 로봇 HOME과 다르다. 실행할 때 로봇별 서술 파일을 여기에 만들어 쓴다.
GENERATED_RMPFLOW_DIR = str(PROJECT_ROOT / "outputs" / "rmpflow")

# usd의 "/World/Cube"는 scale=(0.2,0.2,0.2)에 extent가 -0.5~0.5인 유닛 큐브라
# 실제 한 변 길이는 0.2m (20cm) 이다. 이제는 크기를 그대로 쓰지 않고 XY 생성
# 위치와 컨베이어 상면 Z만 템플릿으로 사용하며, 실제 치수는 아래 RANDOM_BOX_*
# 범위에서 매 실행 랜덤으로 뽑는다.
CUBE_SIZE = 0.2
# 박스 물리는 이식 명세서 §5.2/§5.3 값이다. 질량은 고정값이 아니라 부피 × 밀도로
# 매번 계산한다 — 1호(0.22×0.19×0.09)가 0.56 kg, 4호(0.41×0.31×0.28)가 5.3 kg으로
# 크기에 따라 10배 차이가 나므로, 예전의 0.1 kg 고정값은 큰 박스를 실제보다
# 훨씬 가볍게 만들어 적재 안정성을 낙관적으로 보이게 했다.
CUBE_DENSITY_KG_M3 = 150.0
# 마찰과 damping도 명세서 값으로 되돌렸다. 직전 값(1.5/1.2, damping 2.0/5.0)은
# 골판지 상자의 실제 마찰보다 훨씬 크고 감쇠도 과해서, 쌓다가 미끄러지거나
# 넘어갔어야 할 배치가 붙어 버틴다. 명세서 §9가 금지한 설정이다.
CUBE_STATIC_FRICTION = 0.45
CUBE_DYNAMIC_FRICTION = 0.35
CUBE_LINEAR_DAMPING = 0.1
CUBE_ANGULAR_DAMPING = 0.1
CUBE_SLEEP_THRESHOLD = 0.01
# 겹친 물체를 밀어내 분리할 때의 최대 속도(m/s). PhysxRigidBodyAPI를 붙이면 이
# 값이 0으로 기록되는데, PhysX는 0을 거부하면서 큐브를 스폰할 때마다
# "maxDepenVel must be greater than zero" 에러를 낸다. 명시적으로 지정해서 없앤다.
CUBE_MAX_DEPENETRATION_VELOCITY = 5.0
FLOOR_Z = 0.0  # 기본 바닥(ground plane) 높이

# ============================================================
# RANDOM BOX SIZE / COLOR SETTINGS
# ============================================================
# 실행할 때마다 박스의 가로/세로/높이를 RANDOM_BOX_SIZE_STEP_M 단위로 랜덤
# 생성한다. 그리퍼 접근 높이와 팔레트 적재 Z는 박스별 실제 높이(dimensions[2])로
# 계산되므로 높이가 달라도 픽/플레이스 좌표는 자동으로 맞춰진다.
RANDOM_BOX_MIN_SIZE = np.array([0.10, 0.10, 0.10], dtype=np.float64)
RANDOM_BOX_MAX_SIZE = np.array([0.24, 0.24, 0.20], dtype=np.float64)
RANDOM_BOX_SIZE_STEP_M = 0.01

# Downloads/reference-20260809T222855Z-1-001의 골판지 외관 설정.
# 박스별 색은 크래프트지 기준색의 각 RGB 채널을 이 범위만큼만
# 흔들어 서로 구분하되 원색 플라스틱 큐브처럼 보이지 않게 한다.
CARDBOARD_BASE_COLOR = np.array([0.62, 0.47, 0.30], dtype=np.float64)
CARDBOARD_COLOR_JITTER = 0.08
CARDBOARD_COLOR_CLIP_RANGE = (0.05, 0.95)

# 참조본의 UsdPreviewSurface 설정. 높은 roughness와 낮은 specular로
# 골판지의 거친 무광 표면을 만들고 금속 반사를 없앤다.
CUBE_MATERIAL_ROUGHNESS = 0.85
CUBE_MATERIAL_SPECULAR = 0.05
CUBE_MATERIAL_METALLIC = 0.0

# ============================================================
# DEEPPACK3D PACKING SETTINGS  (test3_vision_check.py에서 이식)
# ============================================================
# 팔레트 중심 기준 고정 격자 대신 DeepPack3D 구성적 휴리스틱으로 적재 순서와
# 좌표(층 쌓기 포함)를 계산한다.
DEEPPACK3D_METHOD = "baf"
DEEPPACK3D_RESOLUTION_M = 0.01
DEEPPACK3D_EDGE_MARGIN_M = 0.015
DEEPPACK3D_BOX_GAP_M = 0.015
DEEPPACK3D_MAX_STACK_HEIGHT_M = 0.8

# 배치 선택기. "default"는 원본 DeepPack3D 그대로, "stable"은 수평 지지 +
# 무게중심 필터를 추가로 건다. 휴리스틱 4종과 직교하므로 --packing-method와
# 자유롭게 조합할 수 있다.
DEEPPACK3D_PLACER = "default"

# --placer stable에서만 쓰는 임계값 두 개.
#
# 수평(측면) 지지 비율: 박스의 -x, -y 옆면이 팔레트 벽이나 이웃 박스와 맞닿는
# 면적 비율이다. 바닥 지지(min_support_ratio)와 직교하는 축이라 값이 겹치지
# 않는다 — 옆에서 받쳐주면 같은 바닥 지지율이어도 훨씬 안 넘어진다.
# arXiv:2307.11531의 수평 지지 정의이고, 0.3은 정완지표 실측에서 쓰인 값이다.
STABLE_MIN_HORIZONTAL_SUPPORT = 0.3
# 무게중심 여유 비율: 박스 중심에서 지지영역 경계까지의 거리를 지지영역의 짧은
# 변으로 나눈 값. 0.5면 지지영역 한가운데, 0.0이면 경계에 정확히 걸린 상태라
# 실물은 넘어가기 직전이다.
STABLE_MIN_COM_MARGIN = 0.05
STABLE_SCORE_TIEBREAK = False
CENTER_STACK_CANDIDATES = False

# 박스를 Z축으로 90도 돌려 놓는 것을 허용할지.
#
# 반쪽 팔레트는 1.003 x 0.485 m로 가늘고 길어서, 회전 없이는 큰 박스의 긴 변이
# 깊이 방향에 들어가지 않는다. 실측(seed 1~10, 40개 투입, stable 0.3/0.05)에서
# bssf가 109 -> 169개, 체적효율 43.9% -> 62.4%로 올랐고 위험률은 그대로였다.
#
# 두 로봇 모두 Pick Zone 밖 측면 staging에서 목표 quaternion을 먼저 완성한 뒤
# 그 자세를 유지해 하강한다. 남쪽 robot_2는 북쪽과 대칭인 -90°를 사용한다.
# 전체 다중 seed 검증 전이므로 기본값은 계속 OFF다.
DEEPPACK3D_ALLOW_YAW_ROTATION = False
# Yaw 박스는 Pick Zone을 벗어난 측면 안전 위치에서 목표 자세를 먼저 완성한다.
# 위치와 quaternion 각도 오차가 모두 이 기준을 연속 만족해야 운반을 시작한다.
YAW_ORIENTATION_TOLERANCE_DEG = 3.0
YAW_ORIENTATION_STABLE_STEPS = 10

# ============================================================
# PALLETIZING PLAN 발행 / 저장 설정
# ============================================================
# 계획이 확정되면 <목표 배치 정보>를 JSON 파일로 남기고 같은 내용을
# std_msgs/msg/String에 실어 한 번 발행한다. info_check.py가 이 토픽을 받는다.
PLAN_JSON_PATH = str(PROJECT_ROOT / "outputs" / "palletizing_plan.json")
PLAN_TOPIC_NAME = "/palletizing/plan"
PLAN_NODE_NAME = "palletizing_plan_publisher"

# H2017 도달 반경. 팔레트에는 이 반경을 넘는 구석이 있으므로, 계획 단계에서
# 적재 영역을 도달 가능한 범위로 미리 잘라낸다. 실행 중에 목표가 범위를 벗어나
# 중단되는 것보다 처음부터 닿는 영역만 쓰는 편이 낫다.
ROBOT_REACH_LIMIT_M = 1.7

# link_6 로컬 +Z가 DOWNWARD_QUAT 적용 시 아래를 향한다고 가정. 그리퍼 끝(흡착
# 접촉면)까지의 거리.
# USD 형상 측정에 실패할 경우에만 사용하는 기존 VG10 길이 fallback.
GRIPPER_TIP_OFFSET_M = 0.14538
GRIPPER_CONTACT_CLEARANCE_M = 0.005
APPROACH_HEIGHT_M = 0.20
# place 완료 뒤와 yaw 회전 전에 사용하는 측면 대기점의 Y 오프셋.
# 컨베이어 중심에서 자기 베이스 쪽으로 0.55 m 벗어나 카메라와 박스 경로를 비운다.
SIDE_STANDBY_LATERAL_OFFSET_M = 0.55
# 팔레트 위에 이미 놓인 큐브보다 충분히 높은 위치에서 XY 이동한 뒤 수직 하강.
PLACE_APPROACH_HEIGHT_M = 0.25
PLACE_CLEARANCE_M = 0.01

# 릴리스 직전/직후 박스 pose 안전 기준. 일반 EE 도달 허용오차(3.5 cm)는
# RMPflow의 전체 3-D 평형 판정용이라 실제 지지면 접촉 판정에는 너무 느슨하다.
# 검증된 단일 릴리스 설정. 계획 지지면 위 6 mm를 목표로 하고 0~12 mm 창에서
# 자세를 확인한 뒤 고정 30 step 동안 안정화를 관찰한다.
PLACE_RELEASE_TARGET_GAP_M = 0.006
PLACE_RELEASE_MIN_GAP_M = 0.0
PLACE_RELEASE_MAX_GAP_M = 0.012
PLACE_RELEASE_MAX_HORIZONTAL_ERROR_M = 0.015
PLACE_RELEASE_MAX_TILT_DEG = 3.5
PLACE_RELEASE_SERVO_INTERVAL_STEPS = 10
# reference의 낮은 RMP gain에서는 현재 EE 기준의 작은 비례 목표가 cspace 힘에
# 밀려 정상상태 오차를 남긴다. 작은 적분 보정을 누적해 천천히 실제 gap을 없앤다.
PLACE_RELEASE_SERVO_GAIN = 0.25
PLACE_RELEASE_SERVO_MAX_STEP_M = 0.005
PLACE_RELEASE_SERVO_MAX_TOTAL_CORRECTION_M = 0.060
PLACE_RELEASE_SERVO_STABLE_STEPS = 10
# 8-seed sweep (46 settled poses): median 0.1 mm, p95 3.0 mm, one upright
# 18 mm 안팎의 정착 drift가 반복되어 전체 적재가 너무 자주 중단되므로 5 mm
# 품질 경고는 유지하되, 강제 종료는 25 mm를 초과할 때만 한다.
BOX_STABILITY_WARN_DRIFT_M = 0.005
BOX_STABILITY_MAX_DRIFT_M = 0.025
BOX_STABILITY_MAX_SUPPORT_HEIGHT_ERROR_M = 0.005
BOX_STABILITY_MAX_TILT_DEG = 2.0

STABILITY_POLICY = "strict"
SUPPORTED_STABILITY_POLICIES = ("strict", "continue-drift")
FREEZE_AFTER_STABILITY = True

# 쿼터니언 순서 [w, x, y, z]. 실제 link_6 축과 다르면 조정 필요.
DOWNWARD_QUAT = np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)

PHYSICS_DT = 1.0 / 60.0
# 이 USD/URDF 조합은 하향 자세를 함께 구속하면 목표 근처 약 2.8 cm에서
# RMPflow가 평형을 이룬다. 흡착면 크기를 고려해 3.5 cm를 도달 판정으로 쓴다.
POSITION_TOLERANCE_M = 0.035
# 적재가 층으로 쌓이면 place approach 지점이 높아져 RMPflow 평형 오차도 커진다.
# 이 지점은 정확히 맞출 필요가 없는 경유점이므로 도달 판정을 느슨하게 둔다.
PLACE_APPROACH_POSITION_TOLERANCE_M = 0.055
# Downloads reference의 안정 팔레타이징 값. 일반 이동은 5 step, 진공 해제 직전은
# 25 step 연속 도달을 요구해 잔류 속도가 박스로 전달되는 것을 줄인다.
STABLE_STEPS = 5
PLACE_DESCEND_STABLE_STEPS = 25

# reference와 동일한 흡착 안정화 시간과 고정 릴리스 관찰 시간이다.
GRAB_SETTLE_STEPS = 10
RELEASE_SETTLE_STEPS = 30

# assets/doosan-robot2/urdf/h2017.urdf의 관절별 속도 한계 (rad/s).
# = 100 / 80 / 100 / 180 / 180 / 225 deg/s. 실제 H2017 스펙 값이다.
#
# RMPFlow의 joint_velocity_cap_rmp는 관절별이 아니라 스칼라 하나(3.927)를
# 쓰므로 joint_1~3에는 실제 한계의 2.2~2.8배를 허용한다. 즉 RMPFlow가
# 관절 한계를 지켜 주지 않는다. 지금 동작이 하드웨어 한계에 얼마나 가까운지
# 알아야 "더 빠르게 할 수 있는가"에 답할 수 있어, 실제 사용률을 측정해 찍는다.
URDF_JOINT_VELOCITY_LIMITS = np.array(
    [1.7453, 1.3963, 1.7453, 3.1416, 3.1416, 3.9270], dtype=np.float64
)
# 같은 URDF의 관절별 토크 한계 (Nm). 속도만 지켜도 토크로 폴트가 나는 경우가
# 있어 함께 잰다. drive의 maxForce는 코드가 건드리지 않으므로(USD 값 그대로)
# 시뮬레이터는 이 한계를 강제하지 않는다 — 넘는지 여부는 측정으로만 알 수 있다.
URDF_JOINT_EFFORT_LIMITS = np.array(
    [372.0, 372.0, 372.0, 163.0, 96.0, 50.0], dtype=np.float64
)
MAX_STEPS_PER_MOVE = 900
HOME_JOINT_TOLERANCE_RAD = 0.02
HOME_JOINT_STEP_RAD = 0.01
HOME_MAX_STEPS = 600

# ============================================================
# 두 로봇 동시 동작 / 충돌 회피
# ============================================================
# 두 팔이 물리적으로 겹칠 수 있는 공유 구역. 이름은 RegionLock의 키로 쓴다.
REGION_PICK_1 = "pick_zone_1"
REGION_PICK_2 = "pick_zone_2"
PICK_REGIONS = (REGION_PICK_1, REGION_PICK_2)
REGION_PALLET_CENTER = "pallet_center"
# 중앙선 양쪽 이 거리 안에 목표 박스 발자국이 걸치면 두 팔의 place 동작을
# 상호배타로 실행한다. 계획상 중앙 여백은 0이고, 이 값은 빈 공간이 아니라
# 로봇 작업 경로를 직렬화하는 인터락 폭이다(전체 폭 0.50 m).
PALLET_CENTER_INTERLOCK_HALF_WIDTH_M = 0.25
# 중앙 인터락 진입 전 payload의 가장 가까운 모서리가 band 밖에서 이만큼 더
# 떨어지도록 staging한다. 잠금을 기다리는 두 로봇은 이 위치에서 동시에 대기한다.
PALLET_CENTER_STAGING_CLEARANCE_M = 0.08

# 구역이 잠겨 있어 기다려야 할 때 물러나 있을 자세. HOME EE 위치를 그대로 쓴다.
# 각 로봇 베이스 쪽이라 두 공유 구역 어디에도 걸치지 않고, 이미 도달이 검증된
# 지점이라 따로 튜닝할 값이 없다.
STANDBY_POSITION_TOLERANCE_M = 0.08

# ============================================================
# BOX SPAWN TIMING
# ============================================================
# 박스는 로봇 상태(HOME 복귀 등)와 무관하게 물리 시간 간격으로 컨베이어에
# 투입된다. 로봇 사이클보다 간격이 짧으면 박스가 컨베이어 위에 줄을 서고,
# 그만큼 로봇은 대기 없이 연속으로 일한다.
# 10.0이던 것을 5.0으로 줄였다. 실측에서 두 로봇의 사이클이 약 8.5s였는데
# 10s 간격으로는 박스가 모자라 로봇이 놀았다(로그의 "대기열=1"이 늘지 않음).
# 두 대가 번갈아 처리하므로 이론상 사이클/2 = 약 4.3s까지 받을 수 있지만,
# 벨트에 박스가 줄을 서면 카메라 시야에 여러 개가 들어와 비전이 엉뚱한 것을
# 재므로(실측: "multiple boxes detected") 포화점보다 여유를 두고 5.0으로 잡는다.
BOX_SPAWN_INTERVAL_SEC = 5.0

# 다만 같은 자리에 겹쳐 생성하면 PhysX가 두 강체를 밀어내며 폭주하므로,
# 직전 박스가 생성 지점에서 이 거리만큼 벗어나기 전에는 스폰을 미룬다.
# (건너뛰지 않고 미루므로 --box-count 개수는 그대로 지켜진다.)
BOX_SPAWN_CLEARANCE_M = 0.30

# ============================================================
# 택배 상자 카탈로그
# ============================================================
# 우체국 택배 상자 규격. 값은 반드시 (긴변, 짧은변, 높이) 순서다.
# spawn_cube가 dimensions를 회전 없이 scale로 적용하므로 dimensions[1]이 그대로
# 벨트 폭 방향 치수가 된다. 벨트 주행면은 0.45 m뿐이라 순서가 뒤집히면 4호가
# 프레임에 부딪힌다.
#
# 5호(0.48 x 0.38 x 0.34)와 6호(0.52 x 0.48 x 0.40)는 제외했다.
#   6호 - 반쪽 팔레트에 50셀이 필요한데 가용은 45셀이다.
#   5호 - 팔레트에는 들어가지만 벨트 폭 여유가 3.5 cm뿐이고, 박스가 벨트보다
#         8.8 cm 위에서 생성되어 낙하하므로 흔들리면 프레임에 닿는다.
BOX_CATALOG: dict[int, tuple[float, float, float]] = {
    1: (0.22, 0.19, 0.09),
    2: (0.27, 0.18, 0.15),
    3: (0.34, 0.25, 0.21),
    4: (0.41, 0.31, 0.28),
}

# 측정 LWH와 카탈로그 항목의 거리가 이 값을 넘으면 미지 박스로 보고 제외한다.
# 카탈로그 항목 사이 최소 거리는 1호-2호의 0.0787이므로, 이 임계값은 최근접
# 판정을 보장하지 않는다. 어디까지나 "명백히 카탈로그 밖"을 걸러내는 용도다.
BOX_SNAP_MAX_DISTANCE_M = 0.05

# 도달 검사(application.py의 probe_z)와 Pick Zone 검사에 쓰는 최대 박스 치수.
# 카탈로그 최대값과 일치해야 한다.
CATALOG_MAX_SIZE = np.array([0.41, 0.31, 0.28], dtype=np.float64)

# 측정 패스에서 비전 노드의 응답을 기다리는 한계. 비전 노드가 떠 있지 않으면
# 여기서 걸려 명확한 에러로 죽는다(무한 대기 금지).
MEASUREMENT_TIMEOUT_SEC = 10.0

# 비전 노드가 모든 내부 프레임 재시도 후에도 box:null을 보냈을 때 측정을 다시
# 요청하는 횟수. 일시적인 카메라 갱신 지연은 복구하되 무한 대기는 하지 않는다.
MEASUREMENT_EMPTY_RETRY_COUNT = 2

# 컨베이어 정지 후 비전 트리거 전 기다리는 물리 step 수. seed 7에서
# 30→25→20→15→10→5→1→0 step을 순차 검증했고, 0 step도 6/6 정확했다.
# seed 13의 10박스 장기 검증에서도 10/10 검출·분류되어 명시적 대기는 뺐다.
MEASUREMENT_SETTLE_STEPS = 0


def parse_box_numbers(value: str) -> tuple[int, ...]:
    """Parse catalog selections such as ``2-4`` or ``1,3,4``."""
    selected: set[int] = set()
    try:
        for raw_token in value.split(","):
            token = raw_token.strip()
            if not token:
                raise ValueError
            if "-" in token:
                bounds = token.split("-")
                if len(bounds) != 2:
                    raise ValueError
                start, end = (int(bound.strip()) for bound in bounds)
                if start > end:
                    raise ValueError
                selected.update(range(start, end + 1))
            else:
                selected.add(int(token))
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "호수는 2-4 또는 1,3,4 형식으로 지정해야 합니다."
        ) from exc

    unknown = sorted(selected.difference(BOX_CATALOG))
    if not selected or unknown:
        available = ",".join(map(str, sorted(BOX_CATALOG)))
        raise argparse.ArgumentTypeError(
            f"사용할 수 없는 박스 호수입니다: {unknown or value} "
            f"(사용 가능: {available})"
        )
    return tuple(sorted(selected))


def parse_args():
    parser = argparse.ArgumentParser(
        description="H2017 랜덤 박스 DeepPack3D 팔레타이징",
    )
    parser.add_argument(
        "--box-count",
        type=int,
        default=DEFAULT_BOX_COUNT,
        help=f"랜덤 생성 및 팔레타이징할 박스 수 (기본: {DEFAULT_BOX_COUNT})",
    )
    parser.add_argument(
        "--box-numbers",
        type=parse_box_numbers,
        default=tuple(sorted(BOX_CATALOG)),
        metavar="RANGE",
        help=(
            "스폰할 박스 호수. 범위(2-4) 또는 목록(1,3,4)으로 지정 "
            f"(기본: {min(BOX_CATALOG)}-{max(BOX_CATALOG)})"
        ),
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="랜덤 시드. 생략하면 실행할 때마다 다른 박스를 생성합니다.",
    )
    parser.add_argument(
        "--spawn-interval",
        type=float,
        default=BOX_SPAWN_INTERVAL_SEC,
        help=(
            "박스 스폰 간격(초, 물리 시간 기준). "
            f"기본: {BOX_SPAWN_INTERVAL_SEC}. 생성 지점이 비어 있지 않으면 "
            "그만큼 미뤄지므로 실제로는 최소 간격이다."
        ),
    )
    parser.add_argument(
        "--measurement-settle-steps",
        type=int,
        default=MEASUREMENT_SETTLE_STEPS,
        help=(
            "컨베이어 정지 후 비전 트리거 전 안정화 step 수 "
            f"(기본: {MEASUREMENT_SETTLE_STEPS})"
        ),
    )
    parser.add_argument(
        "--packing-method",
        choices=("bl", "baf", "bssf", "blsf"),
        default=DEEPPACK3D_METHOD,
        help=f"DeepPack3D 구성적 휴리스틱 (기본: {DEEPPACK3D_METHOD})",
    )
    parser.add_argument(
        "--stability-policy",
        choices=SUPPORTED_STABILITY_POLICIES,
        default=STABILITY_POLICY,
        help=(
            "strict: drift >25 mm 즉시 중단. continue-drift: support/tilt가 "
            "정상이면 degraded로 기록하고 계속 진행"
        ),
    )
    parser.add_argument(
        "--freeze-after-stability",
        action=argparse.BooleanOptionalAction,
        default=FREEZE_AFTER_STABILITY,
        help=(
            "dynamic 정착과 안정성 PASS 뒤 박스를 kinematic 지지물로 고정한다. "
            "비교 실험에서는 --no-freeze-after-stability로 끈다"
        ),
    )
    parser.add_argument(
        "--exit-on-complete",
        action="store_true",
        help="적재 안정성 요약 출력 직후 종료한다. 반복 스윕/CI 실행용이다.",
    )
    parser.add_argument(
        "--run-summary-path",
        default=None,
        help="완료 지표를 저장할 JSON 경로. UI launcher가 실행별 임시 경로를 전달한다.",
    )
    parser.add_argument(
        "--placer",
        choices=("default", "stable"),
        default=DEEPPACK3D_PLACER,
        help=(
            "default: 원본 DeepPack3D(수직 지지만 검사). "
            "stable: 수평 지지 + 무게중심 필터 추가. "
            f"휴리스틱 선택(--packing-method)과는 별개다 (기본: {DEEPPACK3D_PLACER})"
        ),
    )
    parser.add_argument(
        "--stable-min-horizontal-support",
        type=float,
        default=STABLE_MIN_HORIZONTAL_SUPPORT,
        help=(
            "--placer stable일 때 요구하는 최소 수평(측면) 지지 비율. -x/-y "
            f"옆면이 벽이나 이웃 박스와 맞닿는 면적 (기본: {STABLE_MIN_HORIZONTAL_SUPPORT})"
        ),
    )
    parser.add_argument(
        "--yaw-rotation",
        action="store_true",
        default=DEEPPACK3D_ALLOW_YAW_ROTATION,
        help=(
            "두 로봇 모두 박스를 Z축 90도로 돌려 놓는다. Pick Zone 밖 측면 "
            "staging에서 손목 yaw를 먼저 완성한 뒤 place까지 자세를 유지한다"
        ),
    )
    parser.add_argument(
        "--stable-tiebreak",
        action="store_true",
        default=STABLE_SCORE_TIEBREAK,
        help="휴리스틱 점수가 같은 적층 후보만 COM/측면 지지로 우선순위화한다",
    )
    parser.add_argument(
        "--center-stack-candidates",
        action="store_true",
        default=CENTER_STACK_CANDIDATES,
        help="기존 free-space 모서리 후보에 지지 박스 중심 적층 후보를 추가한다",
    )
    parser.add_argument(
        "--stable-min-com-margin",
        type=float,
        default=STABLE_MIN_COM_MARGIN,
        help=(
            "--placer stable일 때 요구하는 최소 무게중심 여유 비율 "
            f"(기본: {STABLE_MIN_COM_MARGIN})"
        ),
    )
    # SimulationApp이 자기 인자를 함께 받으므로 알 수 없는 인자는 무시한다.
    args, unknown = parser.parse_known_args()
    removed_options = {
        "--arm-obstacles",
        "--reconcile-settled-pose",
        "--stable-score-first",
    }
    used_removed = sorted(
        {
            token.split("=", 1)[0]
            for token in unknown
            if token.split("=", 1)[0] in removed_options
        }
    )
    if used_removed:
        parser.error(
            "검증 결과 제거된 옵션입니다: " + ", ".join(used_removed)
        )
    if args.box_count < 1:
        parser.error("--box-count는 1 이상이어야 합니다.")
    if args.spawn_interval <= 0.0:
        parser.error("--spawn-interval은 0보다 커야 합니다.")
    if args.measurement_settle_steps < 0:
        parser.error("--measurement-settle-steps는 0 이상이어야 합니다.")
    if not 0.0 <= args.stable_min_horizontal_support <= 1.0:
        parser.error("--stable-min-horizontal-support는 0~1 범위여야 합니다.")
    if not -1.0 <= args.stable_min_com_margin <= 1.0:
        parser.error("--stable-min-com-margin은 -1~1 범위여야 합니다.")
    return args

# ============================================================
# CONVEYOR PICK ZONE / SPEED CONTROL SETTINGS
# ============================================================
# 큐브 "중심"의 월드 좌표가 이 직육면체 범위 안에 들어오면 컨베이어를 정지한다.
# 실제 컨베이어 끝 좌표에 맞게 반드시 수정한다.
CONVEYOR_PICK_ZONE_MINS = (
    np.array([0.15, -0.90, 0.0], dtype=np.float64),
    np.array([0.15, 1.75, 0.0], dtype=np.float64),
)
CONVEYOR_PICK_ZONE_MAXS = (
    np.array([0.35, 0.0, 2.0], dtype=np.float64),
    np.array([0.35, 2.15, 2.0], dtype=np.float64),
)
# 순수 로직의 기존 단일 라인 테스트/API 호환용 별칭.
PICK_ZONE_MIN = CONVEYOR_PICK_ZONE_MINS[0]
PICK_ZONE_MAX = CONVEYOR_PICK_ZONE_MAXS[0]

# Pick Zone 감지 최대 대기 시간과 로그 출력 주기.
PICK_ZONE_TIMEOUT_SEC = 30.0
PICK_ZONE_LOG_INTERVAL_STEPS = 60

# 컨베이어 정지 후 큐브가 미끄러지지 않고 안정화되도록 기다리는 step 수.
PICK_ZONE_SETTLE_STEPS = 30

# 컨베이어 속도 Attribute의 전체 경로. 이 장면의 컨베이어는
# ConveyorTrack_02(x -2.5~-1.5) → _01(-1.5~-0.5) → ConveyorTrack(-0.5~0.5)
# 세 구간이 이어진 한 줄이다. 마지막 구간이 박스를 Pick Zone까지 밀어 넣으므로
# 셋을 함께 세워야 한다.
#
# 속도는 ConveyorNode의 입력이 아니라 ConveyorBeltGraph의 OmniGraph 변수에
# 들어 있다. USD를 바꿔 경로가 달라지면 시작하자마자 죽으므로, 그때
# [CONVEYOR] 로그에 찍히는 실제 경로로 이 목록을 고치면 된다.
# USD에 저장된 진행 방향은 유지하고 속력만 이 값으로 덮어쓴다.
CONVEYOR_RUN_SPEED_MPS = 1.2
CONVEYOR_SPEED_ATTRIBUTE_PATHS: tuple[tuple[str, ...], ...] = (
    (
        "/World/ConveyorTrack/ConveyorBeltGraph.graph:variable:Velocity",
        "/World/ConveyorTrack_01/ConveyorBeltGraph.graph:variable:Velocity",
        "/World/ConveyorTrack_02/ConveyorBeltGraph.graph:variable:Velocity",
    ),
    (
        "/World/ConveyorTrack_04/ConveyorBeltGraph.graph:variable:Velocity",
        "/World/ConveyorTrack_05/ConveyorBeltGraph.graph:variable:Velocity",
        "/World/ConveyorTrack_06/ConveyorBeltGraph.graph:variable:Velocity",
    ),
)

# v8 USD에 저장된 두 카메라 Action Graph의 실제 ROS2 토픽.
CAMERA_DEPTH_TOPICS = ("/cv1_depth", "/cv2_depth")
CAMERA_INFO_TOPICS = ("/cv_camera1_info", "/cv_camera2_info")
CONVEYOR_TRIGGER_TOPICS = ("/conveyor_1/status", "/conveyor_2/status")
VISION_DETECTION_TOPICS = (
    "/vision/conveyor_1/box_detections",
    "/vision/conveyor_2/box_detections",
)
