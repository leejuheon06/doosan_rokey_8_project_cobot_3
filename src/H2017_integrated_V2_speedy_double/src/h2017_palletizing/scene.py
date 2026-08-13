
"""USD scene inspection, setup, and box/pallet helpers."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from .config import *


def use_local_scene_assets(stage) -> None:
    """Replace online pallet/conveyor/camera references with bundled local assets.

    The collected v6 stage stores these references as public HTTP URLs. Replacing
    them after opening the root layer makes subsequent composition independent of
    network availability. The conveyor camera is kept alive because the box
    measurement node consumes its depth stream; the remaining cameras are
    deactivated so they cost no render time.
    """
    pallet = stage.GetPrimAtPath(PALLET_PRIM_PATH)
    if pallet.IsValid():
        pallet.GetPayloads().ClearPayloads()
        pallet.GetPayloads().AddPayload(LOCAL_PALLET_USD)

    for prim_path in (*CONVEYOR_1_PRIM_PATHS, *CONVEYOR_2_PRIM_PATHS):
        conveyor = stage.GetPrimAtPath(prim_path)
        if not conveyor.IsValid():
            continue
        conveyor.GetReferences().ClearReferences()
        conveyor.GetReferences().AddReference(LOCAL_CONVEYOR_USD)

    for prim_path in LOCAL_CAMERA_PRIM_PATHS:
        camera = stage.GetPrimAtPath(prim_path)
        if not camera.IsValid():
            continue
        camera.SetActive(True)
        camera.GetPayloads().ClearPayloads()
        camera.GetPayloads().AddPayload(LOCAL_RSD455_USD)

    for prim_path in OPTIONAL_CAMERA_PRIM_PATHS:
        camera = stage.GetPrimAtPath(prim_path)
        if camera.IsValid():
            camera.SetActive(False)

    for prim_path in OPTIONAL_CAMERA_GRAPH_PATHS:
        graph = stage.GetPrimAtPath(prim_path)
        if graph.IsValid():
            graph.SetActive(False)

    print(
        f"[로컬 자산] pallet={LOCAL_PALLET_USD}, "
        f"conveyors={LOCAL_CONVEYOR_USD}, "
        f"cameras={LOCAL_RSD455_USD}"
    )


def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None

def get_world_position(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim을 찾을 수 없습니다: {prim_path}")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    p = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([p[0], p[1], p[2]], dtype=np.float64)

def get_box_pose_sample(prim_path: str):
    """Read center, local up axis, and actual AABB bottom for stability checks."""
    from .stability import BoxPoseSample

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim을 찾을 수 없습니다: {prim_path}")
    transform = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    center = transform.ExtractTranslation()
    up = transform.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
    up_array = np.asarray(up, dtype=np.float64)
    up_norm = float(np.linalg.norm(up_array))
    if up_norm > 1e-9:
        up_array /= up_norm
    bottom_z = float(get_world_aabb(prim_path)[2])
    return BoxPoseSample(
        tuple(float(v) for v in center),
        tuple(float(v) for v in up_array),
        bottom_z,
    )

def get_world_aabb(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim을 찾을 수 없습니다: {prim_path}")
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    )
    aligned_box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    minimum = np.asarray(aligned_box.GetMin(), dtype=np.float64)
    maximum = np.asarray(aligned_box.GetMax(), dtype=np.float64)
    return np.concatenate((minimum, maximum))

def random_box_size(rng) -> np.ndarray:
    """RANDOM_BOX_MIN/MAX_SIZE 사이에서 치수를 RANDOM_BOX_SIZE_STEP_M 단위로 뽑는다.

    실수 범위에서 바로 뽑지 않고 step 단위 격자(cell)로 뽑는 이유는 1 cm 같은
    깔끔한 값이 나와야 로그와 팔레트 슬롯 계산을 눈으로 검산하기 쉽기 때문이다.
    rng.integers의 상한은 배타적이므로 +1 해서 최대 크기까지 포함시킨다.
    """
    min_cells = np.rint(RANDOM_BOX_MIN_SIZE / RANDOM_BOX_SIZE_STEP_M).astype(int)
    max_cells = np.rint(RANDOM_BOX_MAX_SIZE / RANDOM_BOX_SIZE_STEP_M).astype(int)
    return (rng.integers(min_cells, max_cells + 1) * RANDOM_BOX_SIZE_STEP_M).astype(
        np.float64
    )

def random_display_color(rng) -> np.ndarray:
    """참조본처럼 크래프트 골판지 기준색 주변에서 RGB를 뽑는다."""
    low, high = CARDBOARD_COLOR_CLIP_RANGE
    return np.clip(
        CARDBOARD_BASE_COLOR
        + rng.uniform(-CARDBOARD_COLOR_JITTER, CARDBOARD_COLOR_JITTER, size=3),
        low,
        high,
    )

def bind_matte_material(
    prim_path: str,
    color: np.ndarray,
    material_path: str,
    binding_strength=UsdShade.Tokens.weakerThanDescendants,
) -> None:
    """프림에 빛이 반사되지 않는 무광 UsdPreviewSurface를 바인딩한다.

    displayColor만 두면 RTX가 기본 재질로 렌더해서 조명 방향에 따라 하이라이트가
    생긴다. 카메라로 색을 검출할 때 그 하이라이트가 흰색으로 날아가 색이 뭉개지므로
    확산 반사만 남는 재질을 직접 붙인다.
    """
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim을 찾을 수 없습니다: {prim_path}")

    material = UsdShade.Material.Define(stage, material_path)
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*(float(v) for v in color))
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(CUBE_MATERIAL_ROUGHNESS)
    shader.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(
            CUBE_MATERIAL_SPECULAR,
            CUBE_MATERIAL_SPECULAR,
            CUBE_MATERIAL_SPECULAR,
        )
    )
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(CUBE_MATERIAL_METALLIC)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, binding_strength)

def configure_cube_appearance(cube_prim_path: str, color: np.ndarray) -> None:
    """큐브에 무광 재질을 입힌다.

    physics 재질(configure_cube_physics)과는 binding purpose가 달라서
    ("physics" vs 기본) 서로 덮어쓰지 않고 함께 적용된다.
    """
    stage = omni.usd.get_context().get_stage()
    cube_prim = stage.GetPrimAtPath(cube_prim_path)
    if not cube_prim.IsValid():
        raise RuntimeError(f"큐브 Prim을 찾을 수 없습니다: {cube_prim_path}")

    # 재질을 큐브와 같은 뿌리 아래 두면 remove_spawned_cubes()가 함께 지운다.
    looks_path = f"{SPAWNED_CUBE_ROOT_PATH}/Looks"
    if not stage.GetPrimAtPath(looks_path).IsValid():
        UsdGeom.Scope.Define(stage, looks_path)

    bind_matte_material(
        cube_prim_path, color, f"{looks_path}/{cube_prim.GetName()}_Matte"
    )

def remove_spawned_cubes() -> None:
    """이 실행에서 만든 큐브들을 timeline 재시작 전에 정리한다."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(SPAWNED_CUBE_ROOT_PATH).IsValid():
        stage.RemovePrim(SPAWNED_CUBE_ROOT_PATH)


def remove_cube(cube_path: str) -> None:
    """큐브 하나만 지운다.

    remove_spawned_cubes()는 Scope 전체를 지우는 all-or-nothing이라 측정 패스에서
    쓸 수 없다. 측정 패스는 박스를 하나 올려 재고 치운 뒤 다음 박스를 올린다.
    """
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(cube_path).IsValid():
        stage.RemovePrim(cube_path)

def spawn_cube(
    spawn_index: int,
    total_box_count: int,
    position: np.ndarray,
    dimensions: np.ndarray,
    color: np.ndarray,
    conveyor_id: int = 1,
) -> str:
    """한 컨베이어에 동적 큐브를 생성한다.

    두 라인의 로컬 순번이 같아도 Prim과 box_id가 충돌하지 않도록 라인 번호를
    이름에 포함한다. 물리·재질·질량 설정은 검증된 단일 라인 구현을 그대로 쓴다.
    """
    if conveyor_id not in (1, 2):
        raise ValueError(f"conveyor_id는 1 또는 2여야 합니다: {conveyor_id}")
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(SPAWNED_CUBE_ROOT_PATH).IsValid():
        UsdGeom.Scope.Define(stage, SPAWNED_CUBE_ROOT_PATH)

    cube_name = f"C{conveyor_id}_Cube_{spawn_index:02d}"
    cube_path = f"{SPAWNED_CUBE_ROOT_PATH}/{cube_name}"
    if stage.GetPrimAtPath(cube_path).IsValid():
        raise RuntimeError(f"생성할 큐브 경로가 이미 존재합니다: {cube_path}")

    # DynamicCuboid는 이 Isaac Sim 버전에서 내부 기본 크기(0.2)에 전달한
    # scale(0.2)을 다시 곱해 0.04 m 큐브를 만들었다. USD Cube를 직접 만들고
    # unit cube(1.0)에 실측 치수를 한 번만 scale로 적용한다.
    cube_geom = UsdGeom.Cube.Define(stage, cube_path)
    cube_geom.CreateSizeAttr(1.0)
    cube_geom.CreateExtentAttr([
        Gf.Vec3f(-0.5, -0.5, -0.5),
        Gf.Vec3f(0.5, 0.5, 0.5),
    ])
    cube_geom.CreateDisplayColorAttr([Gf.Vec3f(*(float(v) for v in color))])
    xform = UsdGeom.XformCommonAPI(cube_geom.GetPrim())
    xform.SetTranslate(Gf.Vec3d(*(float(v) for v in position)))
    xform.SetScale(Gf.Vec3f(*(float(v) for v in dimensions)))

    UsdPhysics.CollisionAPI.Apply(cube_geom.GetPrim())
    rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube_geom.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr(True)
    rigid_body.CreateKinematicEnabledAttr(False)
    mass_api = UsdPhysics.MassAPI.Apply(cube_geom.GetPrim())
    # 질량은 실제 치수에서 계산한다(명세서 §5.2: mass = W×D×H×150).
    cube_mass = float(np.prod(dimensions)) * CUBE_DENSITY_KG_M3
    mass_api.CreateMassAttr(cube_mass)
    configure_cube_physics(cube_path)
    configure_cube_appearance(cube_path, color)
    spawned_aabb = get_world_aabb(cube_path)
    spawned_dimensions = spawned_aabb[3:] - spawned_aabb[:3]
    if not np.allclose(spawned_dimensions, dimensions, atol=0.002):
        raise RuntimeError(
            f"생성 큐브 크기 불일치: expected={dimensions}, actual={spawned_dimensions}"
        )
    print(
        f"[SPAWN {spawn_index}/{total_box_count}] {cube_path}: "
        f"center={np.round(position, 3)}, size={np.round(spawned_dimensions, 3)}, "
        f"mass={cube_mass:.3f} kg, color={np.round(color, 3)}"
    )
    return cube_path

def build_plan_payload(
    placements, destination_centers, box_dimensions, pallet_aabb
) -> list[dict]:
    """계획된 적재 정보를 JSON으로 내보낼 수 있는 구조로 바꾼다.

    좌표계는 팔레트 기준이다. x/y는 팔레트 중심에서의 오프셋이고, bottom_z는
    팔레트 윗면부터 박스 바닥까지의 높이다. 월드 절대좌표 대신 이 기준을 쓰면
    팔레트를 옮겨도 같은 계획이 그대로 유효하다.

    width/length/height는 yaw를 적용하기 전 박스 고유 치수다. 회전은 yaw로 따로
    알려 주므로 받는 쪽이 필요할 때 직접 적용한다.

    ROS나 시뮬레이터 상태에 의존하지 않는 순수 계산이라 따로 검산하기 쉽다.
    """
    pallet_aabb = np.asarray(pallet_aabb, dtype=np.float64)
    pallet_center_xy = (pallet_aabb[:2] + pallet_aabb[3:5]) * 0.5
    pallet_top_z = float(pallet_aabb[5])

    def rounded(value) -> float:
        """0.1 mm 단위로 끊어 읽기 좋게 만든다.

        마지막에 0.0을 더하는 것은 -0.0을 0.0으로 바꾸기 위해서다. 1층 박스처럼
        결과가 0에 아주 가까우면 round()가 -0.0을 내놓는데, 값은 같아도 JSON에
        "-0.0"으로 찍혀 읽는 사람이 부호를 의심하게 된다.
        """
        return round(float(value), 4) + 0.0

    payload = []
    for placement in placements:
        box_id = placement.item.item_id
        dimensions = np.asarray(box_dimensions[box_id], dtype=np.float64)
        # 실행 중 목표에 더해지는 PLACE_CLEARANCE_M은 넣지 않는다. 그건 놓을 때의
        # 여유이지 알고리즘이 계획한 목표 자체가 아니다.
        center = np.asarray(destination_centers[box_id], dtype=np.float64)
        payload.append(
            {
                "id": int(placement.sequence_index),
                "width": rounded(dimensions[0]),
                "length": rounded(dimensions[1]),
                "height": rounded(dimensions[2]),
                "x": rounded(center[0] - pallet_center_xy[0]),
                "y": rounded(center[1] - pallet_center_xy[1]),
                "bottom_z": rounded(center[2] - dimensions[2] * 0.5 - pallet_top_z),
                "yaw": float(placement.yaw_degrees),
            }
        )
    return payload

def save_plan_json(payload: list[dict], path: str) -> None:
    """계획 정보를 사람이 읽을 수 있는 JSON 파일로 남긴다."""
    Path(path).write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"[PLAN 저장] {path} (박스 {len(payload)}개)")

def setup_scene_physics():
    """현재 열려 있는 USD Stage에 로봇과 무관한 물리 설정을 적용한다.

    USD 자체는 main() 시작부에서 open_stage()로 전체 Stage를 연다.
    따라서 Action Graph와 /World 밖의 Prim도 그대로 유지된다.
    로봇별(drive/VG10/EE) 설정은 RobotUnit.setup_physics()가 담당한다.
    """
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("열린 USD Stage가 없습니다.")

    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        raise RuntimeError("저장된 USD에 /World Prim이 없습니다.")

    for cube_path in CONVEYOR_TEMPLATE_CUBE_PATHS:
        if not stage.GetPrimAtPath(cube_path).IsValid():
            raise RuntimeError(f"🚨 템플릿 큐브 Prim을 찾을 수 없습니다: {cube_path}")

    pallet_prim = stage.GetPrimAtPath(PALLET_PRIM_PATH)
    if not pallet_prim.IsValid():
        raise RuntimeError(f"🚨 pallet Prim을 찾을 수 없습니다: {PALLET_PRIM_PATH}")
    original_pallet_position = get_world_position(PALLET_PRIM_PATH)
    original_pallet_aabb = get_world_aabb(PALLET_PRIM_PATH)
    # 현재 Prim 원점과 실제 형상 바닥면 사이의 오프셋을 보존하면서 AABB min Z를
    # 정확히 바닥 높이로 이동한다.
    pallet_origin_z_on_floor = (
        original_pallet_position[2]
        + FLOOR_Z
        - original_pallet_aabb[2]
    )
    pallet_world_position = np.array([
        PALLET_WORLD_XY[0],
        PALLET_WORLD_XY[1],
        pallet_origin_z_on_floor,
    ])
    # pallet은 기존 xform op stack이 XformCommonAPI와 호환되지 않으므로,
    # 현재 회전/스케일을 보존한 matrix op로 변환한 뒤 translation만 바꾼다.
    pallet_xformable = UsdGeom.Xformable(pallet_prim)
    pallet_local_matrix = pallet_xformable.GetLocalTransformation()
    pallet_local_matrix.SetTranslateOnly(
        Gf.Vec3d(*(float(value) for value in pallet_world_position))
    )
    pallet_xformable.MakeMatrixXform().Set(pallet_local_matrix)
    placed_pallet_aabb = get_world_aabb(PALLET_PRIM_PATH)
    if abs(float(placed_pallet_aabb[2]) - FLOOR_Z) > 0.002:
        raise RuntimeError(
            f"팔레트 바닥 정렬 실패: min_z={placed_pallet_aabb[2]:.4f}, "
            f"ground_z={FLOOR_Z:.4f}"
        )
    print(f"  [OK] pallet 원점 위치 적용: {np.round(pallet_world_position, 4)}")
    print(
        f"  [OK] pallet 바닥면 Z: {placed_pallet_aabb[2]:.4f} m "
        f"(ground={FLOOR_Z:.4f} m)"
    )

def configure_cube_physics(cube_prim_path: str) -> None:
    """큐브의 미끄러짐과 미세 진동을 억제하는 PhysX 속성을 적용한다."""
    stage = omni.usd.get_context().get_stage()
    cube_prim = stage.GetPrimAtPath(cube_prim_path)
    if not cube_prim.IsValid():
        raise RuntimeError(f"큐브 Prim을 찾을 수 없습니다: {cube_prim_path}")

    material_path = "/World/PhysicsMaterials/CubeHighFriction"
    material = UsdShade.Material.Define(stage, material_path)
    material_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    material_api.CreateStaticFrictionAttr(CUBE_STATIC_FRICTION)
    material_api.CreateDynamicFrictionAttr(CUBE_DYNAMIC_FRICTION)
    material_api.CreateRestitutionAttr(0.0)
    physx_material = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx_material.CreateFrictionCombineModeAttr("max")
    physx_material.CreateRestitutionCombineModeAttr("min")

    binding_api = UsdShade.MaterialBindingAPI.Apply(cube_prim)
    binding_api.Bind(
        material,
        UsdShade.Tokens.weakerThanDescendants,
        "physics",
    )

    physx_body = PhysxSchema.PhysxRigidBodyAPI.Apply(cube_prim)
    physx_body.CreateLinearDampingAttr(CUBE_LINEAR_DAMPING)
    physx_body.CreateAngularDampingAttr(CUBE_ANGULAR_DAMPING)
    physx_body.CreateSleepThresholdAttr(CUBE_SLEEP_THRESHOLD)
    physx_body.CreateStabilizationThresholdAttr(CUBE_SLEEP_THRESHOLD)
    physx_body.CreateMaxDepenetrationVelocityAttr(CUBE_MAX_DEPENETRATION_VELOCITY)
    print(
        f"  [OK] 큐브 물리 안정화: static friction={CUBE_STATIC_FRICTION}, "
        f"dynamic friction={CUBE_DYNAMIC_FRICTION}"
    )

def configure_pallet_as_fixed_support(pallet_prim_path: str) -> None:
    """팔레트를 움직이지 않는 kinematic 충돌 지지물로 설정한다."""
    stage = omni.usd.get_context().get_stage()
    pallet_root = stage.GetPrimAtPath(pallet_prim_path)
    if not pallet_root.IsValid():
        raise RuntimeError(f"팔레트 Prim을 찾을 수 없습니다: {pallet_prim_path}")

    rigid_body_count = 0
    for prim in Usd.PrimRange(pallet_root):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        rigid_body.CreateKinematicEnabledAttr(True).Set(True)
        rigid_body.CreateVelocityAttr(Gf.Vec3f(0.0))
        rigid_body.CreateAngularVelocityAttr(Gf.Vec3f(0.0))
        rigid_body_count += 1
    print(f"  [OK] pallet 고정 완료 (rigid bodies={rigid_body_count})")


def freeze_settled_box(box_prim_path: str) -> int:
    """Make a validated settled box kinematic while keeping collision enabled."""
    stage = omni.usd.get_context().get_stage()
    box_root = stage.GetPrimAtPath(box_prim_path)
    if not box_root.IsValid():
        raise RuntimeError(f"고정할 박스 Prim을 찾을 수 없습니다: {box_prim_path}")
    rigid_body_count = 0
    for prim in Usd.PrimRange(box_root):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rigid_body = UsdPhysics.RigidBodyAPI(prim)
        rigid_body.CreateVelocityAttr(Gf.Vec3f(0.0)).Set(Gf.Vec3f(0.0))
        rigid_body.CreateAngularVelocityAttr(Gf.Vec3f(0.0)).Set(Gf.Vec3f(0.0))
        rigid_body.CreateKinematicEnabledAttr(True).Set(True)
        rigid_body_count += 1
    if rigid_body_count == 0:
        raise RuntimeError(f"고정할 RigidBody가 없습니다: {box_prim_path}")
    print(f"[SETTLED FIX] {box_prim_path} (rigid bodies={rigid_body_count})")
    return rigid_body_count
