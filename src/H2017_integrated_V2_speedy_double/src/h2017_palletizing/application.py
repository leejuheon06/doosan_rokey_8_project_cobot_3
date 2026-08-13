
"""Top-level orchestration for the standalone H2017 palletizer."""

from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np
import omni.usd
from isaacsim.core.api import World

from .config import *
from .conveyor import BoxSpawner, ConveyorGate, ConveyorSpeedController
from .coordination import (
    RegionLock,
    center_interlock_staging_box_y,
    conveyor_side_standby_target,
    footprint_intersects_center_interlock,
)
from .efficiency import calculate_space_efficiency
from .planning import PackingItem, make_planner, rotate_quaternion_about_world_z
from .robot import RobotUnit
from .ros_bridge import PalletizingRosBridge
from .intake import (
    IntakeStation,
    Measurement,
    MeasurementPassError,
    report_accuracy,
)
from .scene import (
    build_plan_payload,
    configure_pallet_as_fixed_support,
    freeze_settled_box,
    get_world_aabb,
    get_box_pose_sample,
    get_world_position,
    random_display_color,
    remove_cube,
    remove_spawned_cubes,
    save_plan_json,
    setup_scene_physics,
    spawn_cube,
    use_local_scene_assets,
)
from .stability import (
    assess_box_pose,
    assess_release_pose,
    is_drift_only_failure,
    pose_motion_rates,
)



class TimelineRestartRequested(RuntimeError):
    pass


def _save_run_summary(path: str | None, payload: dict) -> None:
    """Atomically save one completed run for the ROS launcher/UI."""
    if not path:
        return
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(destination)
    print(f"[RUN SUMMARY] {destination}", flush=True)


def wait_until_playing(world: World, simulation_app) -> bool:
    """Render while paused and report whether the app is still running.

    Timeline Stop followed by Play is a reset request.  Closing the window is
    different: Kit can invalidate physics handles during the same frame in
    which ``is_running()`` changes to false, so callers must stop immediately.
    """
    timeline_was_stopped = False
    while not world.is_playing() and simulation_app.is_running():
        timeline_was_stopped = timeline_was_stopped or world.is_stopped()
        world.render()
        time.sleep(0.05)
    if not simulation_app.is_running():
        return False
    if timeline_was_stopped and simulation_app.is_running():
        raise TimelineRestartRequested("Timeline Stop 후 Play가 감지되었습니다.")
    return True


def step_world(
    world: World,
    attachments,
    simulation_app,
    spawners,
    conveyor_gates,
    render: bool = True,
) -> bool:
    """Advance one physics step and return false when Kit starts shutting down."""
    if not wait_until_playing(world, simulation_app):
        return False
    for attachment in attachments:
        if not simulation_app.is_running():
            return False
        attachment.update()
    if not simulation_app.is_running():
        return False
    world.step(render=render)
    if not wait_until_playing(world, simulation_app):
        return False
    for attachment in attachments:
        if not simulation_app.is_running():
            return False
        attachment.update()
    if not simulation_app.is_running():
        return False
    for spawner in spawners:
        spawner.tick(PHYSICS_DT)
    for conveyor_gate in conveyor_gates:
        conveyor_gate.update()
    return simulation_app.is_running()



def run(simulation_app):
    args = parse_args()
    box_count = args.box_count

    for _ in range(10):
        simulation_app.update()
    # 기존 빈 Stage에 USD를 Reference로 붙이지 않고,
    # TF/카메라 Action Graph가 저장된 USD 전체 Stage를 직접 연다.
    usd_context = omni.usd.get_context()

    print(f"[USD OPEN] {USD_PATH}")
    open_result = usd_context.open_stage(USD_PATH)
    if open_result is False:
        raise RuntimeError(f"USD Stage 열기 요청 실패: {USD_PATH}")

    # Isaac Sim 5.1 환경에서는 is_stage_loading()이 없을 수 있으므로
    # update를 반복하면서 필수 Prim이 나타나는지 확인한다.
    units = [RobotUnit(config) for config in ROBOT_CONFIGS]
    required_paths = [
        "/World",
        PALLET_PRIM_PATH,
        *CONVEYOR_TEMPLATE_CUBE_PATHS,
        *CONVEYOR_1_PRIM_PATHS,
        *CONVEYOR_2_PRIM_PATHS,
        *[unit.root_path for unit in units],
    ]
    stage = None
    for _ in range(300):
        simulation_app.update()
        stage = usd_context.get_stage()
        if stage is not None and all(
            stage.GetPrimAtPath(path).IsValid() for path in required_paths
        ):
            break
    else:
        raise RuntimeError(
            f"USD Stage 로드 시간 초과: {required_paths}를 확인하세요."
        )

    use_local_scene_assets(stage)
    for _ in range(10):
        simulation_app.update()

    # Action Graph가 실제 Stage에 포함되었는지 로그로 확인한다.
    graph_paths = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if "graph" in prim.GetName().lower()
    ]
    print(f"[USD OPEN 완료] graph prims={graph_paths}")

    world = World(
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )

    # 저장된 USD에 GroundPlane이 이미 있으므로 add_default_ground_plane()은 호출하지 않는다.
    setup_scene_physics()
    for unit in units:
        unit.setup_physics()
        unit.add_to_world(world)

    # 기존 큐브는 위치만 템플릿으로 사용하고 삭제한다. 첫 큐브는 reset 전에
    # 생성하여 PhysX가 초기 장면 구성 때부터 인식하게 한다.
    stage = omni.usd.get_context().get_stage()
    for unit in units:
        unit.base_pos = get_world_position(unit.base_link_path)
        print(f"[로봇 베이스] {unit.name}: {np.round(unit.base_pos, 3)}")
    cube_spawn_template_centers = []
    source_support_zs = []
    for conveyor_index, cube_path in enumerate(CONVEYOR_TEMPLATE_CUBE_PATHS):
        cube_aabb = get_world_aabb(cube_path)
        center = (cube_aabb[:3] + cube_aabb[3:]) * 0.5
        dimensions = cube_aabb[3:] - cube_aabb[:3]
        if not np.all(np.isfinite(dimensions)) or np.any(dimensions <= 0.0):
            raise RuntimeError(f"C{conveyor_index + 1} 템플릿 AABB 오류: {cube_aabb}")
        cube_spawn_template_centers.append(center)
        source_support_zs.append(float(cube_aabb[2]))
        # 참조 레이어의 Prim은 RemovePrim 대신 active override로 확실히 숨긴다.
        stage.GetPrimAtPath(cube_path).SetActive(False)
        print(
            f"[템플릿 C{conveyor_index + 1}] center={np.round(center, 3)}, "
            f"size={np.round(dimensions, 3)}, support_z={cube_aabb[2]:.3f}"
        )
    for _ in range(2):
        simulation_app.update()
    for cube_path in CONVEYOR_TEMPLATE_CUBE_PATHS:
        if stage.GetPrimAtPath(cube_path).IsActive():
            raise RuntimeError(f"템플릿 큐브 비활성화 실패: {cube_path}")

    configure_pallet_as_fixed_support(PALLET_PRIM_PATH)
    pallet_pos = get_world_position(PALLET_PRIM_PATH)
    pallet_aabb = get_world_aabb(PALLET_PRIM_PATH)

    pallet_center = (pallet_aabb[:3] + pallet_aabb[3:]) * 0.5
    pallet_size = pallet_aabb[3:] - pallet_aabb[:3]

    print(f"[PALLET CENTER] {np.round(pallet_center, 4)}")
    print(f"[PALLET SIZE]   {np.round(pallet_size, 4)}")
    print(f"[PALLET AABB]   {np.round(pallet_aabb, 4)}")

    # ------------------------------------------------------------------
    # 팔레트를 두 로봇 사이(중앙 y)로 반씩 나눠, 각 로봇이 자기 반쪽에만 쌓는다.
    #
    # 잠금은 "같은 지점에 동시에 들어가지 않는다"만 보장할 뿐, 이동 중 팔이
    # 스치는 것까지는 못 막는다. 영역을 아예 갈라 두면 두 팔이 서로의 구역으로
    # 넘어갈 일이 없어져 훨씬 강한 기하학적 분리가 된다. 각 로봇은 자기 베이스와
    # 같은 쪽 반쪽을 맡으므로, 팔이 분할선을 넘는 것은 컨베이어에서 집을 때뿐이고
    # 그건 Pick Zone 잠금이 막는다.
    # ------------------------------------------------------------------
    pallet_mid_y = float((pallet_aabb[1] + pallet_aabb[4]) * 0.5)
    for unit in units:
        half = pallet_aabb.copy()
        if float(unit.base_pos[1]) >= pallet_mid_y:
            half[1] = pallet_mid_y      # 로봇이 북쪽 -> 북쪽 절반
        else:
            half[4] = pallet_mid_y      # 로봇이 남쪽 -> 남쪽 절반
        unit.half_aabb = half

    if len({float(u.half_aabb[1]) for u in units}) != len(units):
        raise RuntimeError(
            "두 로봇이 팔레트 같은 쪽에 있어 영역을 나눌 수 없습니다: "
            f"{[np.round(u.base_pos, 3).tolist() for u in units]}"
        )
    # 적재 영역 도달검사는 그리퍼 오프셋을 실측한 뒤에 한다(아래 참조).

    # 브리지는 측정 패스보다 먼저 있어야 한다. 계획 발행은 계획이 나온 뒤이지만,
    # 검출 구독은 첫 박스를 재기 전에 붙어 있어야 측정을 놓치지 않는다.
    ros = PalletizingRosBridge()
    ros.subscribe_detections()

    # KinematicAttachment가 실행 중 이 프림의 kinematic 상태와 Xform을 직접
    # 변경한다. world.scene에 등록하면 해당 변경 시 tensor view 전체(로봇 포함)가
    # 무효화되므로, 물리 프림만 다루고 Scene tensor 객체로는 등록하지 않는다.
    world.reset()
    for unit in units:
        unit.initialize()

    # 로봇이 도착하기 전까지, 이미 usd에 배치된 큐브는 별도 조치 없이도
    # (일반 dynamic 강체로) 그 자리에 가만히 서 있는다.
    for _ in range(30):
        world.step(render=True)

    # 로봇마다 HOME 자세가 다르므로 그리퍼 접촉 오프셋도 각자 측정한다.
    for unit in units:
        unit.measure_gripper_contact_offset()
        unit.create_controller()

    # 적재 영역 도달검사. 그리퍼 오프셋 실측 뒤에 해야 추정값이 아닌 실제 값으로
    # 판단한다(생성자 기본값은 fallback이라 3 cm 크다).
    #
    # 온라인에서는 배치를 되돌릴 수 없으므로(PackingSession.place()가 상태를
    # 확정한다), 이 반쪽에서 EE가 갈 수 있는 가장 높은 지점 — 최대 적층 높이의
    # 박스를 놓기 직전 place approach — 까지 시작 시점에 한 번에 보장한다.
    # 여기서 통과하면 실행 중 도달 실패가 나지 않는다.
    for unit in units:
        probe_z = (
            pallet_aabb[5]
            + DEEPPACK3D_MAX_STACK_HEIGHT_M
            + PLACE_CLEARANCE_M
            + unit.gripper_contact_offset
            + PLACE_APPROACH_HEIGHT_M
        )
        worst = max(
            float(np.linalg.norm(np.array([cx, cy, probe_z]) - unit.base_pos))
            for cx in (unit.half_aabb[0], unit.half_aabb[3])
            for cy in (unit.half_aabb[1], unit.half_aabb[4])
        )
        if worst > ROBOT_REACH_LIMIT_M:
            raise RuntimeError(
                f"{unit.name}이 자기 적재 영역에 닿지 않습니다: "
                f"최원거리={worst:.3f} m, 한계={ROBOT_REACH_LIMIT_M} m. "
                f"DEEPPACK3D_MAX_STACK_HEIGHT_M을 낮추거나 팔레트를 로봇 쪽으로 "
                f"옮기세요."
            )
        print(
            f"[적재 영역] {unit.name}: y {unit.half_aabb[1]:.3f}~{unit.half_aabb[4]:.3f} "
            f"({unit.half_aabb[3] - unit.half_aabb[0]:.3f} x "
            f"{unit.half_aabb[4] - unit.half_aabb[1]:.3f} m), "
            f"최원거리 {worst:.3f} m (probe_z={probe_z:.3f} m)"
        )

    conveyors = [
        ConveyorSpeedController(paths, f"conveyor_{index + 1}")
        for index, paths in enumerate(CONVEYOR_SPEED_ATTRIBUTE_PATHS)
    ]
    for conveyor in conveyors:
        conveyor.start()

    # 로봇마다 HOME 자세가 달라 HOME EE 위치/자세도 서로 다르다. 각자 저장해
    # 두고 그 로봇의 모든 이동에서 같은 자세를 쓴다. 잠금 대기 시 물러나는
    # 대기 자세로도 이 위치를 쓴다.
    for unit in units:
        unit.capture_home_ee_pose()

    # ------------------------------------------------------------------
    # 온라인 입고 — 박스를 하나 스폰해 재고, 그 자리에서 배치를 결정해 그
    # 박스를 그대로 적재한다. 사전 계획도, 전수 측정 패스도 없다.
    #
    # 플래너는 로봇마다 세션 하나를 들고, 한 번에 한 개씩만 본다(K=1). 명세서
    # §6이 요구하는 온라인 입고 조건이다 — 플래너가 순서를 바꾸거나 뒤 박스를
    # 먼저 집을 수 없다.
    # ------------------------------------------------------------------
    rng = np.random.default_rng(args.seed)
    # 참조본처럼 색상 난수열을 박스 호수 선택과 분리한다. 재질 표현을
    # 바꿔도 같은 --seed의 입고 순서가 바뀌지 않아 실험을 비교할 수 있다.
    color_rng = np.random.default_rng(None if args.seed is None else args.seed + 1)
    catalog_numbers = list(args.box_numbers)

    def make_session(unit):
        north_side = float(unit.base_pos[1]) >= pallet_mid_y
        return make_planner(
            unit.half_aabb,
            placer=args.placer,
            min_horizontal_support_ratio=args.stable_min_horizontal_support,
            min_com_margin_ratio=args.stable_min_com_margin,
            stability_tiebreak=args.stable_tiebreak,
            center_stack_candidates=args.center_stack_candidates,
            method=args.packing_method,
            lookahead=1,
            resolution=DEEPPACK3D_RESOLUTION_M,
            max_stack_height=DEEPPACK3D_MAX_STACK_HEIGHT_M,
            edge_margin=DEEPPACK3D_EDGE_MARGIN_M,
            # 팔레트 외곽은 15 mm를 유지하고 중앙 분할선만 여백을 없앤다.
            min_y_margin=0.0 if north_side else DEEPPACK3D_EDGE_MARGIN_M,
            max_y_margin=DEEPPACK3D_EDGE_MARGIN_M if north_side else 0.0,
            # 격자 y=0이 두 로봇 모두 중앙선이 되도록 남쪽 세션만 뒤집는다.
            reverse_y=not north_side,
            box_gap=DEEPPACK3D_BOX_GAP_M,
            min_support_ratio=0.5,
            # 두 로봇 모두 측면 staging에서 yaw를 먼저 완성한 뒤 그 자세를
            # 고정해 하강한다. 남쪽 robot_2는 아래에서 대칭인 -90°를 사용한다.
            allow_yaw_rotation=args.yaw_rotation,
        ).open_session()

    sessions = [make_session(unit) for unit in units]
    if args.yaw_rotation:
        print(
            "[플래너] yaw rotation ON — 두 로봇 모두 측면 staging에서 "
            "yaw 자세를 먼저 완성한 뒤 place까지 고정",
            flush=True,
        )
    if args.placer == "stable":
        print(
            f"[플래너] stable — 측면 지지 >= {args.stable_min_horizontal_support:.2f}, "
            f"무게중심 여유 >= {args.stable_min_com_margin:.2f}, "
            f"tie-break={'COM' if args.stable_tiebreak else '기존'} "
            f"(휴리스틱 {args.packing_method})",
            flush=True,
        )
    else:
        print(f"[플래너] default (휴리스틱 {args.packing_method})", flush=True)
    print(
        f"[릴리스] target={PLACE_RELEASE_TARGET_GAP_M * 1000.0:.1f} mm, "
        f"window={PLACE_RELEASE_MIN_GAP_M * 1000.0:.1f}~"
        f"{PLACE_RELEASE_MAX_GAP_M * 1000.0:.1f} mm, settle=30 step fixed",
        flush=True,
    )
    print(f"[안정성 정책] {args.stability_policy}", flush=True)
    print(
        f"[안착 후 고정] {'ON' if args.freeze_after_stability else 'OFF'}",
        flush=True,
    )
    stations = [
        IntakeStation(
            conveyors[index],
            ros.line(index + 1),
            settle_steps=args.measurement_settle_steps,
        )
        for index in range(2)
    ]
    print(
        f"[비전 안정화] {args.measurement_settle_steps} step "
        f"({args.measurement_settle_steps * PHYSICS_DT:.2f}s)",
        flush=True,
    )

    truth_numbers: dict[str, int] = {}
    measurements: list[Measurement] = []
    measured_dimensions: dict[str, np.ndarray] = {}
    planned_destination_centers: dict[str, np.ndarray] = {}
    box_owner: dict[str, int] = {}
    box_supports: dict[str, tuple] = {}
    box_paths: dict[str, str] = {}
    ordered_placements: list = []
    removed_count = 0
    pending_stability_checks: dict[str, dict] = {}
    stability_results = []

    # 각 라인은 가장 가까운 전용 로봇이 담당한다.
    for index, unit in enumerate(units):
        zone_min = CONVEYOR_PICK_ZONE_MINS[index]
        zone_max = CONVEYOR_PICK_ZONE_MAXS[index]
        pick_probe = np.array([
            float((zone_min[0] + zone_max[0]) * 0.5),
            float((zone_min[1] + zone_max[1]) * 0.5),
            source_support_zs[index] + float(CATALOG_MAX_SIZE[2]) * 0.5,
        ])
        distance = float(np.linalg.norm(pick_probe - unit.base_pos))
        if distance > ROBOT_REACH_LIMIT_M:
            raise RuntimeError(
                f"{unit.name}이 Pick Zone에 닿지 않습니다: "
                f"pick={np.round(pick_probe, 3)}, 거리={distance:.3f} m"
            )
        print(f"[Pick Zone 도달] C{index + 1} -> {unit.name}: {distance:.3f} m")

    conveyor_box_counts = ((box_count + 1) // 2, box_count // 2)

    def make_spawn_function(conveyor_index: int):
        conveyor_id = conveyor_index + 1
        template_center = cube_spawn_template_centers[conveyor_index]
        support_z = source_support_zs[conveyor_index]
        total_count = conveyor_box_counts[conveyor_index]

        def spawn_next_box(index: int) -> str:
            number = int(rng.choice(catalog_numbers))
            dimensions = np.array(BOX_CATALOG[number], dtype=np.float64)
            position = np.array([
                template_center[0],
                template_center[1],
                support_z + float(dimensions[2]) * 0.5,
            ])
            path = spawn_cube(
                index + 1,
                total_count,
                position,
                dimensions,
                random_display_color(color_rng),
                conveyor_id=conveyor_id,
            )
            truth_numbers[path.rsplit("/", 1)[-1]] = number
            return path

        return spawn_next_box

    print(
        f"\n[온라인 입고 시작] 총 {box_count}개 — "
        f"C1={conveyor_box_counts[0]}, C2={conveyor_box_counts[1]}, "
        f"카탈로그 {catalog_numbers}호, method={args.packing_method}",
        flush=True,
    )

    spawners = [
        BoxSpawner(
            make_spawn_function(index),
            conveyor_box_counts[index],
            args.spawn_interval,
            spawn_position=cube_spawn_template_centers[index],
            name=f"conveyor_{index + 1}",
        )
        for index in range(2)
    ]
    for spawner in spawners:
        spawner.prime()
    for conveyor in conveyors:
        conveyor.start()


    # 안전 1단계: Pick Zone과 팔레트 중앙 교차구역을 각각 상호배타로 만든다.
    # 중앙 바깥의 place는 잠그지 않으므로 두 로봇의 병렬성은 유지된다.
    region_lock = RegionLock([*PICK_REGIONS, REGION_PALLET_CENTER])

    placed_boxes: set = set()

    def build_cycle_steps(
        unit, conveyor_index, cube_path, box_id, source_center,
        destination_center, yaw_degrees
    ):
        """한 박스를 집어 적재하고 컨베이어 측면 대기점으로 가는 단계를 만든다.

        두 로봇이 이 함수를 그대로 공유한다. 로봇에 따라 달라지는 값(그리퍼
        오프셋, HOME 자세)은 unit에서 읽는다. 반환값은 RobotUnit이 매 물리
        step마다 한 칸씩 밀어 주는 단계 목록이라, 두 대가 동시에 진행된다.

        공유 구역(Pick Zone, 팔레트)은 lock/unlock 단계로 감싸 한 번에 한 대만
        들어가게 한다. 팔레트 잠금은 "지지 박스가 다 놓였는지" 확인한 뒤에
        잡는다 — 순서를 뒤집으면 못 놓는 로봇이 잠금을 쥔 채 지지 박스를 든
        상대를 막아 교착에 빠진다.
        """
        conveyor = conveyors[conveyor_index]
        pick_region = PICK_REGIONS[conveyor_index]
        offset = unit.gripper_contact_offset
        # 두 로봇의 HOME 손목 자세는 중앙선을 기준으로 거울 대칭이다. 직육면체의
        # +90/-90도 AABB는 같으므로 각자 바깥쪽으로 도는 대칭 방향을 쓴다.
        signed_yaw_degrees = (
            yaw_degrees
            if float(unit.base_pos[1]) >= pallet_mid_y
            else -yaw_degrees
        )
        place_orientation = rotate_quaternion_about_world_z(
            unit.home_ee_orientation, signed_yaw_degrees
        )
        box_half = float(measured_dimensions[box_id][2] * 0.5)
        planned_support_z = float(destination_center[2]) - box_half
        source_contact = np.array([
            source_center[0], source_center[1],
            source_center[2] + box_half + offset,
        ])
        source_approach = source_contact + np.array([0.0, 0.0, APPROACH_HEIGHT_M])
        release_box_center = np.asarray(destination_center, dtype=np.float64).copy()
        release_box_center[2] += PLACE_RELEASE_TARGET_GAP_M
        destination_contact = np.array([
            release_box_center[0], release_box_center[1],
            release_box_center[2] + box_half + offset,
        ])
        destination_approach = destination_contact + np.array(
            [0.0, 0.0, PLACE_APPROACH_HEIGHT_M]
        )
        # 한 사이클이 끝난 뒤 팔을 컨베이어 중심선에서 자기 베이스 쪽으로
        # 물린다. 다음 박스가 벨트를 따라 지나갈 때 링크를 관통하지 않고,
        # 탑뷰 카메라 시야도 가리지 않는 컨베이어 측면 대기 위치다.
        post_place_standby = conveyor_side_standby_target(
            source_center,
            unit.base_pos,
            float(CATALOG_MAX_SIZE[2]),
            unit.gripper_contact_offset,
            APPROACH_HEIGHT_M,
            SIDE_STANDBY_LATERAL_OFFSET_M,
        )
        alignment_state = {
            "ticks": 0,
            "nominal": destination_contact.copy(),
            "correction": np.zeros(3, dtype=np.float64),
            "grip_offset": np.zeros(3, dtype=np.float64),
            "box_target": np.asarray(destination_center, dtype=np.float64).copy(),
        }
        oriented_depth = (
            float(measured_dimensions[box_id][0])
            if int(yaw_degrees) % 180 == 90
            else float(measured_dimensions[box_id][1])
        )
        # 중앙 인터락에 들어가는 payload는 양쪽 로봇이 동시에 접근 가능한
        # band 바깥 staging을 먼저 거친다. grab 뒤 실측 grip_offset으로 다시
        # 보정되므로 이 배열은 destination_contact처럼 mutable하게 유지한다.
        staging_box_y = center_interlock_staging_box_y(
            float(unit.base_pos[1]),
            pallet_mid_y,
            oriented_depth,
            PALLET_CENTER_INTERLOCK_HALF_WIDTH_M,
            PALLET_CENTER_STAGING_CLEARANCE_M,
        )
        center_staging = destination_approach.copy()
        center_staging[1] = staging_box_y
        uses_center_interlock = footprint_intersects_center_interlock(
            float(destination_center[1]),
            oriented_depth,
            pallet_mid_y,
            PALLET_CENTER_INTERLOCK_HALF_WIDTH_M,
        )

        def grab():
            unit.attachment.grab(cube_path)
            # 실제 흡착 위치의 TCP→박스 중심 오프셋을 배치 목표에 반영한다.
            # 기존 계산은 완벽히 중앙에서 집었다고 가정해 pick 오차가 place 오차로
            # 그대로 복사됐다. 이 보정은 박스를 순간이동하지 않고 로봇 목표만
            # 수정한다.
            box_center = np.asarray(get_box_pose_sample(cube_path).center)
            ee_center = np.asarray(unit.controller.get_end_effector_position())
            grip_offset = ee_center - box_center
            alignment_state["grip_offset"][:] = grip_offset
            destination_contact[:] = release_box_center + grip_offset
            destination_approach[:] = destination_contact + np.array(
                [0.0, 0.0, PLACE_APPROACH_HEIGHT_M]
            )
            center_staging[:] = destination_approach
            center_staging[1] = staging_box_y + grip_offset[1]
            alignment_state["nominal"] = destination_contact.copy()
            alignment_state["correction"][:] = 0.0
            destination_contact[:] = (
                alignment_state["nominal"] + alignment_state["correction"]
            )
            arrived = pick_arrival_times.pop(box_id, None)
            if arrived is not None:
                delay = spawners[conveyor_index].elapsed_sec - arrived
                pick_delays.append(float(delay))
                print(
                    f"[PICK 지연] {box_id}: Pick Zone 도착→GRAB "
                    f"{delay:.2f}s ({delay / PHYSICS_DT:.0f} step)",
                    flush=True,
                )

        def release_status():
            sample = get_box_pose_sample(cube_path)
            reference = np.asarray(destination_center, dtype=np.float64).copy()
            reference[:2] = alignment_state["box_target"][:2]
            return assess_release_pose(
                box_id,
                sample,
                reference,
                float(measured_dimensions[box_id][2]),
                min_gap_m=PLACE_RELEASE_MIN_GAP_M,
                max_gap_m=PLACE_RELEASE_MAX_GAP_M,
                max_horizontal_error_m=PLACE_RELEASE_MAX_HORIZONTAL_ERROR_M,
                max_tilt_deg=PLACE_RELEASE_MAX_TILT_DEG,
            )

        def update_release_target():
            alignment_state["ticks"] += 1
            if alignment_state["ticks"] % PLACE_RELEASE_SERVO_INTERVAL_STEPS:
                return
            sample = get_box_pose_sample(cube_path)
            center = np.asarray(sample.center, dtype=np.float64)
            error = np.array([
                float(alignment_state["box_target"][0]) - center[0],
                float(alignment_state["box_target"][1]) - center[1],
                PLACE_RELEASE_TARGET_GAP_M
                - (sample.bottom_z - planned_support_z),
            ])
            correction = np.clip(
                error * PLACE_RELEASE_SERVO_GAIN,
                -PLACE_RELEASE_SERVO_MAX_STEP_M,
                PLACE_RELEASE_SERVO_MAX_STEP_M,
            )
            nominal = alignment_state["nominal"]
            total = alignment_state["correction"]
            total[:] = np.clip(
                total + correction,
                -PLACE_RELEASE_SERVO_MAX_TOTAL_CORRECTION_M,
                PLACE_RELEASE_SERVO_MAX_TOTAL_CORRECTION_M,
            )
            destination_contact[:] = nominal + total

        def release():
            result = release_status()
            print(
                f"[릴리스 검사] {box_id}: gap="
                f"{result.support_height_error_m * 1000.0:.1f} mm, "
                f"XY={result.horizontal_drift_m * 1000.0:.1f} mm, "
                f"tilt={result.tilt_deg:.2f}° — "
                f"{'PASS' if result.passed else 'FAIL'}",
                flush=True,
            )
            if not result.passed:
                raise RuntimeError(
                    f"{box_id} 안전하지 않은 릴리스 차단: {', '.join(result.reasons)}"
                )
            release_sample = get_box_pose_sample(cube_path)
            unit.attachment.release()
            pending_stability_checks[box_id] = {
                "cube_path": cube_path,
                "release_center": tuple(map(float, release_sample.center)),
                "previous_sample": release_sample,
                "steps": 0,
            }

        def supports_ready():
            return all(s in placed_boxes for s in box_supports[box_id])

        def placement_settled():
            return box_id in placed_boxes

        tol = POSITION_TOLERANCE_M
        needs_yaw_alignment = not np.isclose(signed_yaw_degrees, 0.0)
        # Yaw가 필요한 박스만 Pick Zone 밖 측면으로 먼저 빠져 회전을 완료한다.
        # 회전이 없는 박스는 기존 직행 경로를 유지해 불필요한 takt 증가를 막는다.
        after_grab_steps = (
            [
                ("move", source_approach, tol, "pick lift"),
                ("move", post_place_standby,
                 PLACE_APPROACH_POSITION_TOLERANCE_M,
                 "payload rotation staging"),
                ("unlock", pick_region, "Pick Zone 반납"),
                ("call", conveyor.start, "컨베이어 재시작"),
                ("pose", post_place_standby, tol,
                 place_orientation, YAW_ORIENTATION_TOLERANCE_DEG,
                 "payload yaw align", YAW_ORIENTATION_STABLE_STEPS),
            ]
            if needs_yaw_alignment
            else [
                ("call", conveyor.start, "컨베이어 재시작"),
                ("move", source_approach, tol, "pick lift"),
            ]
        )
        pick_unlock_before_center = (
            [("unlock", pick_region, "Pick Zone 반납")]
            if uses_center_interlock and not needs_yaw_alignment
            else []
        )
        pick_unlock_after_approach = (
            [] if needs_yaw_alignment or uses_center_interlock
            else [("unlock", pick_region, "Pick Zone 반납")]
        )
        supports_gate = (
            ("gate", supports_ready, f"{box_id} 지지 박스 대기", "hold")
            if needs_yaw_alignment or uses_center_interlock
            else ("gate", supports_ready, f"{box_id} 지지 박스 대기")
        )
        center_entry_steps = (
            [
                ("move", center_staging, PLACE_APPROACH_POSITION_TOLERANCE_M,
                 "center interlock staging", STABLE_STEPS, place_orientation),
                *pick_unlock_before_center,
                supports_gate,
                ("lock", REGION_PALLET_CENTER, "팔레트 중앙 교차구역 확보",
                 center_staging, place_orientation),
            ]
            if uses_center_interlock
            else [supports_gate]
        )
        center_exit_steps = (
            [
                ("move", center_staging, PLACE_APPROACH_POSITION_TOLERANCE_M,
                 "center interlock exit", STABLE_STEPS, place_orientation),
                ("unlock", REGION_PALLET_CENTER, "팔레트 중앙 교차구역 반납"),
                ("move", post_place_standby,
                 PLACE_APPROACH_POSITION_TOLERANCE_M,
                 "post-place side standby", STABLE_STEPS),
            ]
            if uses_center_interlock
            else [
                ("move", post_place_standby,
                 PLACE_APPROACH_POSITION_TOLERANCE_M,
                 "post-place side standby", STABLE_STEPS),
            ]
        )
        return [
            # --- 컨베이어에서 집기 (Pick Zone 점유) ---
            ("lock", pick_region, "Pick Zone 확보"),
            ("move", source_approach, tol, "pick approach"),
            ("move", source_contact, tol, "pick descend"),
            ("call", grab, "grab"),
            ("settle", GRAB_SETTLE_STEPS, "grab settle"),
            *after_grab_steps,
            # --- 팔레트에 놓기 (팔레트 점유) ---
            # 중앙 배치는 양쪽 staging까지 병렬 이동한 뒤 잠금을 얻은 로봇만
            # 진입한다. 지지 박스 gate는 잠금보다 먼저라 교착도 피한다.
            *center_entry_steps,
            ("move", destination_approach, PLACE_APPROACH_POSITION_TOLERANCE_M,
             "place approach", STABLE_STEPS, place_orientation),
            # Yaw 박스는 측면 회전 위치에서 이미 Pick Zone을 반납했다. 회전이
            # 없는 박스는 팔레트 상공에 도착한 지금 반납한다.
            *pick_unlock_after_approach,
            # 놓기 직전만 안정 확인을 길게 본다. 흔들린 채 릴리스하면 박스가
            # 튀어 적재가 무너진다.
            ("move", destination_contact, tol, "place descend",
             PLACE_DESCEND_STABLE_STEPS, place_orientation),
            ("servo", destination_contact, update_release_target, release_status,
             "place release align", PLACE_RELEASE_SERVO_STABLE_STEPS,
             place_orientation),
            ("call", release, "release"),
            ("gate", placement_settled, f"{box_id} release settle", "hold"),
            ("move", destination_approach, PLACE_APPROACH_POSITION_TOLERANCE_M,
             "place retreat", STABLE_STEPS, place_orientation),
            # payload footprint 기준 band 밖 staging에 도착하면 즉시 잠금을
            # 반납한다. 이후 컨베이어 대기점 복귀는 다음 로봇의 place와 병렬이다.
            *center_exit_steps,
            # 여기서 사이클이 끝나므로 다음 박스가 아직 없어도 팔은 컨베이어
            # 옆 안전 위치에서 계속 대기한다.
        ]

    # 여기서부터 step_world()가 매 물리 step마다 스폰 타이머와 Pick Zone 정지
    # 판정을 함께 돌린다. (준비용 world.step() 동안에는 시간이 흐르지 않도록
    # 여기서 뒤늦게 연결한다.)
    conveyor_gates = [
        ConveyorGate(
            conveyors[index],
            spawners[index],
            CONVEYOR_PICK_ZONE_MINS[index],
            CONVEYOR_PICK_ZONE_MAXS[index],
            name=f"conveyor_{index + 1}",
        )
        for index in range(2)
    ]
    print(
        f"[스폰 타이머] 간격={args.spawn_interval:.1f}s (물리 시간), "
        f"생성 지점 여유={BOX_SPAWN_CLEARANCE_M:.2f} m, 총 {box_count}개"
    )

    # ------------------------------------------------------------------
    # 병렬 스케줄러
    #
    # 물리 step 하나마다: 두 로봇이 각자 명령을 넣고 -> 물리 1 step ->
    # 두 로봇이 각자 도달 여부를 판정한다. 블로킹 함수가 없으므로 한 대가
    # 적재하는 동안 다른 대가 컨베이어에서 집는 식으로 겹쳐서 돈다.
    # ------------------------------------------------------------------
    attachments = [unit.attachment for unit in units]
    completed_count = 0
    task_complete = False
    cycle_takts: list[int] = []
    robot_cycle_takts: dict[str, list[int]] = {unit.name: [] for unit in units}
    pick_arrival_times: dict[str, float] = {}
    pick_delays: list[float] = []

    def discard_front(conveyor_index: int, reason: str) -> None:
        """한 라인의 앞 박스를 치우고 다른 라인에는 영향을 주지 않는다."""
        nonlocal removed_count
        spawner = spawners[conveyor_index]
        station = stations[conveyor_index]
        conveyor = conveyors[conveyor_index]
        path = spawner.pop_front()
        if path is None:
            return
        box_id = path.rsplit("/", 1)[-1]
        pick_arrival_times.pop(box_id, None)
        remove_cube(path)
        removed_count += 1
        station.reset()
        conveyor.start()
        print(f"[제거 C{conveyor_index + 1}] {path}: {reason}", flush=True)

    def publish_progress() -> None:
        """지금까지 확정된 배치를 JSON과 토픽으로 다시 낸다.

        온라인이라 전체 계획이 미리 없다. 한 개 놓을 곳이 정해질 때마다 누적본을
        새로 낸다. 중간에 죽어도 그때까지의 결과가 남는다.
        """
        payload = build_plan_payload(
            ordered_placements,
            planned_destination_centers,
            measured_dimensions,
            pallet_aabb,
        )
        for global_id, entry in enumerate(payload):
            # sequence_index는 로봇별로 매겨져 중복된다(0,0,1,1,...).
            # 투입 순서 기준 전역 번호로 덮어쓴다.
            entry["id"] = global_id
        save_plan_json(payload, PLAN_JSON_PATH)
        ros.publish_plan(payload)

    def try_assign_work(conveyor_index: int):
        """C1→robot_1, C2→robot_2로 측정과 적재를 독립 진행한다."""
        conveyor = conveyors[conveyor_index]
        spawner = spawners[conveyor_index]
        gate = conveyor_gates[conveyor_index]
        station = stations[conveyor_index]
        unit = units[conveyor_index]
        session = sessions[conveyor_index]
        front_path = spawner.front()
        # 벨트 정지만으로는 부족하다. 로봇이 집는 동안에도 벨트는 서므로,
        # 앞 박스가 실제로 Pick Zone 안에 들어와 있는지 좌표로 확인한다.
        at_pick_zone = False
        if front_path is not None:
            try:
                at_pick_zone = gate.contains(get_world_position(front_path))
            except RuntimeError:
                at_pick_zone = False
        station.update(front_path, at_pick_zone)
        if at_pick_zone and front_path is not None:
            box_id = front_path.rsplit("/", 1)[-1]
            pick_arrival_times.setdefault(box_id, spawner.elapsed_sec)
        if station.state == IntakeStation.WAITING:
            return

        box_id = front_path.rsplit("/", 1)[-1]

        if station.state == IntakeStation.FAILED:
            line_prefix = f"C{conveyor_index + 1}_"
            if station.timed_out and not any(
                measurement.box_id.startswith(line_prefix)
                for measurement in measurements
            ):
                raise MeasurementPassError(
                    f"C{conveyor_index + 1} 첫 박스 측정에 응답이 없습니다 "
                    f"({MEASUREMENT_TIMEOUT_SEC}s 초과). "
                    f"conveyor_box_measurement_node.py가 떠 있는지 확인하세요."
                )
            discard_front(
                conveyor_index,
                "측정 무응답" if station.timed_out else "검출/스냅 실패"
            )
            return

        # 검출 성공 여부와 적재 가능 여부는 서로 다른 지표다. 이전에는 실제
        # 배치가 확정된 뒤에만 measurements에 넣어서, 팔레트 공간 부족으로
        # 제거된 정상 검출 박스가 최종 통계에서 '미검출'로 잘못 집계됐다.
        if not any(item.box_id == box_id for item in measurements):
            measurements.append(
                Measurement(
                    box_id,
                    station.number,
                    station.dimensions,
                    truth_numbers[box_id],
                )
            )

        if unit.busy:
            return

        packing_item = PackingItem(box_id, station.dimensions)
        placement = session.place(packing_item)
        if placement is None:
            discard_front(
                conveyor_index,
                f"{unit.name} 담당 팔레트 반쪽에 둘 곳 없음",
            )
            return

        destination = np.asarray(placement.world_center, dtype=np.float64)
        measured_dimensions[box_id] = np.array(station.dimensions, dtype=np.float64)
        planned_destination_centers[box_id] = destination
        box_owner[box_id] = conveyor_index
        box_supports[box_id] = tuple(placement.support_item_ids)
        box_paths[box_id] = front_path
        ordered_placements.append(placement)
        publish_progress()

        conveyor.stop()
        cube_pos = get_world_position(front_path)
        spawner.pop_front()
        mark = "O" if station.number == truth_numbers[box_id] else "X"
        print(
            f"[배정 C{conveyor_index + 1}] {box_id} -> {unit.name} "
            f"({station.number}호로 분류, 실제 {truth_numbers[box_id]}호 {mark}, "
            f"pick={np.round(cube_pos, 3)}, place={np.round(destination, 3)}, "
            f"yaw={placement.yaw_degrees}°, "
            f"지지={list(placement.support_item_ids) or '바닥'})",
            flush=True,
        )
        unit.begin_cycle(
            build_cycle_steps(
                unit, conveyor_index, front_path, box_id, cube_pos, destination,
                placement.yaw_degrees,
            ),
            box_id,
            len(ordered_placements),
        )
        station.reset()

    while simulation_app.is_running():
        try:
            if not task_complete:
                for conveyor_index in range(2):
                    try_assign_work(conveyor_index)

                for unit in units:
                    unit.apply_action(region_lock)

                if not step_world(
                    world, attachments, simulation_app, spawners,
                    conveyor_gates
                ):
                    print("[종료] SimulationApp 종료 감지 — 로봇 상태 읽기를 중단합니다.")
                    break

                for box_id, settle_state in list(pending_stability_checks.items()):
                    cube_path = settle_state["cube_path"]
                    sample = get_box_pose_sample(cube_path)
                    linear_speed, tilt_rate = pose_motion_rates(
                        settle_state["previous_sample"], sample, PHYSICS_DT
                    )
                    settle_state["previous_sample"] = sample
                    settle_state["steps"] += 1
                    if settle_state["steps"] < RELEASE_SETTLE_STEPS:
                        continue

                    # XY는 릴리스 순간 대비 이동량을, Z는 계획 지지면을 기준으로
                    # 고정 30-step 뒤 채점한다.
                    stability_reference = np.asarray(
                        planned_destination_centers[box_id], dtype=np.float64
                    ).copy()
                    release_center = settle_state["release_center"]
                    stability_reference[:2] = release_center[:2]
                    result = assess_box_pose(
                        box_id,
                        sample,
                        stability_reference,
                        float(measured_dimensions[box_id][2]),
                        max_horizontal_drift_m=BOX_STABILITY_MAX_DRIFT_M,
                        max_support_height_error_m=(
                            BOX_STABILITY_MAX_SUPPORT_HEIGHT_ERROR_M
                        ),
                        max_tilt_deg=BOX_STABILITY_MAX_TILT_DEG,
                    )
                    stability_results.append(result)
                    del pending_stability_checks[box_id]
                    drift_warning = (
                        result.passed
                        and result.horizontal_drift_m > BOX_STABILITY_WARN_DRIFT_M
                    )
                    verdict = (
                        "WARNING" if drift_warning
                        else "PASS" if result.passed
                        else "FAIL"
                    )
                    print(
                        f"[적재 안정성] {box_id}: drift="
                        f"{result.horizontal_drift_m * 1000.0:.1f} mm, "
                        f"support error={result.support_height_error_m * 1000.0:.1f} mm, "
                        f"tilt={result.tilt_deg:.2f}°, "
                        f"settle={settle_state['steps']} step "
                        f"(v={linear_speed * 1000.0:.1f} mm/s, "
                        f"ω={tilt_rate:.2f}°/s, fixed) — "
                        f"{verdict}",
                        flush=True,
                    )
                    if drift_warning:
                        print(
                            f"[적재 안정성 경고] {box_id}: XY drift "
                            f"{result.horizontal_drift_m * 1000.0:.1f} mm > "
                            f"{BOX_STABILITY_WARN_DRIFT_M * 1000.0:.1f} mm "
                            f"(강제 종료 {BOX_STABILITY_MAX_DRIFT_M * 1000.0:.1f} mm)",
                            flush=True,
                        )
                    if not result.passed:
                        drift_only = is_drift_only_failure(
                            result,
                            trigger_drift_m=BOX_STABILITY_MAX_DRIFT_M,
                            max_support_height_error_m=(
                                BOX_STABILITY_MAX_SUPPORT_HEIGHT_ERROR_M
                            ),
                            max_tilt_deg=BOX_STABILITY_MAX_TILT_DEG,
                        )
                        if args.stability_policy == "continue-drift" and drift_only:
                            print(
                                f"[적재 안정성 DEGRADED] {box_id}: drift hard "
                                "기준 초과지만 support/tilt 정상 — 실제 pose로 계속",
                                flush=True,
                            )
                        else:
                            raise RuntimeError(
                                f"{box_id} 적재 안정성 실패: "
                                f"{', '.join(result.reasons)}"
                            )
                    if args.freeze_after_stability and result.passed:
                        freeze_settled_box(cube_path)
                    placed_boxes.add(box_id)

                for unit in units:
                    was_busy = unit.busy
                    unit.observe(region_lock)
                    if was_busy and not unit.busy:
                        unit.completed_count += 1
                        completed_count += 1
                        cycle_takts.append(unit.cycle_steps)
                        robot_cycle_takts[unit.name].append(unit.cycle_steps)
                        print(
                            f"[DONE] {unit.name} cycle {unit.cycle_number} "
                            f"({unit.box_id}) 완료 — 누적 {completed_count}/{box_count}, "
                            f"takt {unit.cycle_steps} step "
                            f"({unit.cycle_steps * PHYSICS_DT:.2f}s)"
                            f"{unit.joint_speed_report()}",
                            flush=True,
                        )
                        unit.task_kind = None

                # 온라인에서는 일부 박스가 버려질 수 있어 단순 카운터로는 끝을
                # 알 수 없다. 다 투입했고, 벨트에 남은 박스도 없고, 두 팔이 다
                # 쉬면 끝이다.
                if (
                    all(spawner.finished for spawner in spawners)
                    and all(spawner.front() is None for spawner in spawners)
                    and not any(unit.busy for unit in units)
                    and not pending_stability_checks
                ):
                    task_complete = True
                    done_counts = {u.name: u.completed_count for u in units}
                    print(
                        f"[DONE] 투입 {box_count} / 적재 {len(ordered_placements)} / "
                        f"제거 {removed_count} {done_counts}. "
                        f"경과 {max(s.elapsed_sec for s in spawners):.1f}s. "
                        "뷰어를 계속 유지합니다.",
                        flush=True,
                    )
                    if cycle_takts:
                        takts = np.array(cycle_takts, dtype=np.float64) * PHYSICS_DT
                        print(
                            f"[TAKT] 사이클 {len(takts)}회: 평균 {takts.mean():.2f}s, "
                            f"최소 {takts.min():.2f}s, 최대 {takts.max():.2f}s "
                            f"(물리 step 기준)",
                            flush=True,
                        )
                    report_accuracy(measurements, box_count)
                    print(
                        f"[적재 안정성 요약] {sum(r.passed for r in stability_results)}/"
                        f"{len(stability_results)} PASS",
                        flush=True,
                    )
                    elapsed_seconds = float(max(s.elapsed_sec for s in spawners))
                    takt_seconds = np.asarray(cycle_takts, dtype=np.float64) * PHYSICS_DT
                    correct = sum(1 for measurement in measurements if measurement.correct)
                    warnings = sum(
                        result.passed
                        and result.horizontal_drift_m > BOX_STABILITY_WARN_DRIFT_M
                        for result in stability_results
                    )
                    drifts_mm = np.asarray(
                        [result.horizontal_drift_m * 1000.0 for result in stability_results],
                        dtype=np.float64,
                    )
                    placed_volume = float(sum(
                        np.prod(measured_dimensions[box_id])
                        for box_id in placed_boxes
                        if box_id in measured_dimensions
                    ))
                    # 분모는 DEEPPACK3D_MAX_STACK_HEIGHT_M이 아니라 이번 실행에서
                    # 실제로 쌓아 올린 높이를 쓴다. 고정 0.8 m를 분모로 두면 지표가
                    # "얼마나 촘촘히 쌓았나"가 아니라 "박스를 몇 개 넣었나"를 재게
                    # 된다 — 박스 5개는 빈틈 없이 쌓아도 10%, 40개는 같은 밀도로
                    # 83%가 나온다. 실제 도달 높이를 쓰면 box_count와 무관하게
                    # 적재 밀도를 비교할 수 있다.
                    stack_top = max(
                        (
                            float(planned_destination_centers[box_id][2])
                            + float(measured_dimensions[box_id][2]) / 2.0
                            for box_id in placed_boxes
                            if box_id in planned_destination_centers
                            and box_id in measured_dimensions
                        ),
                        default=float(pallet_aabb[5]),
                    )
                    achieved_height = max(stack_top - float(pallet_aabb[5]), 1e-6)
                    pallet_capacity = float(
                        (pallet_aabb[3] - pallet_aabb[0])
                        * (pallet_aabb[4] - pallet_aabb[1])
                        * achieved_height
                    )
                    placed_placements = [
                        placement
                        for placement in ordered_placements
                        if placement.item.item_id in placed_boxes
                    ]
                    stability_pass_count = int(sum(r.passed for r in stability_results))
                    space_efficiency = calculate_space_efficiency(
                        (
                            (placement.world_center, placement.oriented_size)
                            for placement in placed_placements
                        ),
                        pallet_aabb,
                        edge_margin_m=DEEPPACK3D_EDGE_MARGIN_M,
                        stability_pass_count=stability_pass_count,
                        stability_check_count=len(stability_results),
                    )
                    per_robot = {}
                    for unit in units:
                        values = (
                            np.asarray(robot_cycle_takts[unit.name], dtype=np.float64)
                            * PHYSICS_DT
                        )
                        per_robot[unit.name] = {
                            "completed": int(unit.completed_count),
                            "average_cycle_seconds": float(values.mean()) if values.size else None,
                            "minimum_cycle_seconds": float(values.min()) if values.size else None,
                            "maximum_cycle_seconds": float(values.max()) if values.size else None,
                        }
                    summary = {
                        "status": "completed",
                        "elapsed_seconds": elapsed_seconds,
                        "input_boxes": int(box_count),
                        "planned_boxes": int(len(ordered_placements)),
                        "placed_boxes": int(len(placed_boxes)),
                        "removed_boxes": int(removed_count),
                        "system_takt_seconds": (
                            elapsed_seconds / len(placed_boxes) if placed_boxes else None
                        ),
                        "throughput_boxes_per_min": (
                            len(placed_boxes) * 60.0 / elapsed_seconds
                            if elapsed_seconds > 0.0 else None
                        ),
                        "takt": {
                            "cycles": int(takt_seconds.size),
                            "average_seconds": float(takt_seconds.mean()) if takt_seconds.size else None,
                            "minimum_seconds": float(takt_seconds.min()) if takt_seconds.size else None,
                            "maximum_seconds": float(takt_seconds.max()) if takt_seconds.size else None,
                        },
                        "per_robot": per_robot,
                        "pick_delay": {
                            "average_seconds": float(np.mean(pick_delays)) if pick_delays else None,
                            "maximum_seconds": float(np.max(pick_delays)) if pick_delays else None,
                        },
                        "vision": {
                            "detected": int(len(measurements)),
                            "detection_rate_percent": (
                                len(measurements) * 100.0 / box_count if box_count else 0.0
                            ),
                            "correct": int(correct),
                            "classification_accuracy_percent": (
                                correct * 100.0 / len(measurements)
                                if measurements else 0.0
                            ),
                        },
                        "stability": {
                            "pass_count": stability_pass_count,
                            "warning_count": int(warnings),
                            "fail_count": int(sum(not r.passed for r in stability_results)),
                            "average_drift_mm": float(drifts_mm.mean()) if drifts_mm.size else None,
                            "maximum_drift_mm": float(drifts_mm.max()) if drifts_mm.size else None,
                        },
                        "packed_volume_m3": placed_volume,
                        "volume_efficiency_percent": (
                            placed_volume * 100.0 / pallet_capacity
                            if pallet_capacity > 0.0 else None
                        ),
                        "space_efficiency": {
                            "footprint_coverage_percent": (
                                space_efficiency.footprint_coverage * 100.0
                            ),
                            "vertical_compactness_percent": (
                                space_efficiency.vertical_compactness * 100.0
                            ),
                            "stability_pass_rate_percent": (
                                space_efficiency.stability_pass_rate * 100.0
                            ),
                            "project_efficiency_percent": (
                                space_efficiency.project_efficiency * 100.0
                            ),
                            "occupied_surface_volume_m3": (
                                space_efficiency.occupied_surface_volume_m3
                            ),
                            "aggregation": "harmonic_mean_times_stability",
                        },
                    }
                    _save_run_summary(args.run_summary_path, summary)
                    if args.exit_on_complete:
                        break
            else:
                if not wait_until_playing(world, simulation_app):
                    break
                world.step(render=True)
        except TimelineRestartRequested:
            print("[TIMELINE] Stop 후 Play 감지 — 로봇/RMPFlow를 재초기화합니다.")
            for unit in units:
                unit.attachment.clear_after_timeline_stop()
                unit.abort_cycle(region_lock)
            region_lock.reset()
            remove_spawned_cubes()
            placed_boxes.clear()
            pending_stability_checks.clear()
            stability_results.clear()
            # 두 스폰 타이머와 대기열도 처음부터 다시 시작한다.
            for spawner in spawners:
                spawner.prime()
            world.reset()
            for unit in units:
                unit.initialize()
                unit.controller.reset()
                unit.completed_count = 0
            for conveyor in conveyors:
                conveyor.start()
            for _ in range(5):
                world.step(render=True)
            for unit in units:
                unit.capture_home_ee_pose()
            # 팔레트 위 박스가 다 지워졌으므로 세션도 새로 연다. 안 그러면
            # 지워진 박스가 아직 있다고 믿고 그 위에 쌓으려 든다.
            for index, unit in enumerate(units):
                sessions[index] = make_session(unit)
            for station in stations:
                station.reset()
            truth_numbers.clear()
            measurements.clear()
            measured_dimensions.clear()
            planned_destination_centers.clear()
            box_owner.clear()
            box_supports.clear()
            box_paths.clear()
            ordered_placements.clear()
            cycle_takts.clear()
            pick_arrival_times.clear()
            removed_count = 0
            completed_count = 0
            task_complete = False
        except RuntimeError:
            # GUI 닫기/Kit 종료는 physics handle을 먼저 해제할 수 있다. 그 짧은
            # 구간에 이미 시작한 observe/FK가 겹치면 RuntimeError가 발생하지만,
            # 앱이 실제로 종료 중인 경우에만 정상 종료로 취급한다. 실행 중의
            # 도달 실패나 물리 오류는 기존처럼 그대로 상위로 전달한다.
            if simulation_app.is_running():
                raise
            print("[종료] SimulationApp 종료 중 physics handle이 해제되었습니다.")
            break
