
"""Robot state machine, collision coordination, and box attachment."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.prims import SingleArticulation

from .config import *
from .coordination import quaternion_angular_error_deg
from .rmpflow_controller import RMPFlowController, make_description_for_home
from .scene import find_prim_path_by_name, get_world_aabb, get_world_position

class RobotUnit:
    """로봇 한 대에 딸린 프림 경로, articulation, 컨트롤러, 부착기를 묶는다.

    두 대가 같은 pick/place 알고리즘을 그대로 공유하고 다른 것은 베이스 위치와
    HOME 자세뿐이다. 로봇마다 달라지는 상태를 전부 이 객체에 모아 두고,
    동작 함수에는 이 객체 하나만 넘겨서 알고리즘 코드가 로봇 수와 무관해진다.

    두 대가 동시에 움직여야 하므로 동작은 블로킹 함수가 아니라 "매 물리 step마다
    한 칸씩 진행하는 단계 목록"으로 표현한다(begin_cycle/apply_action/observe).
    """

    def __init__(self, config: dict) -> None:
        prefix = config["prefix"]
        self.name = config["name"]
        self.root_path = f"{prefix}/h2017_gripper/h2017"
        self.base_link_path = f"{self.root_path}/base_link"
        self.ee_prim_path = f"{self.root_path}/link_6"
        self.vg10_path = f"{prefix}/h2017_gripper/VG10_tool"
        self.home_pose = np.asarray(config["home_pose"], dtype=np.float64)

        # setup 단계에서 채운다.
        self.ee_path = self.ee_prim_path
        self.robot = None
        self.controller = None
        self.attachment = None
        self.base_pos = None
        self.gripper_contact_offset = (
            GRIPPER_TIP_OFFSET_M + GRIPPER_CONTACT_CLEARANCE_M
        )
        self.home_ee_position = None
        self.home_ee_orientation = None
        self.completed_count = 0

        # --- 단계 실행 상태 (두 대가 동시에 돌아가므로 블로킹하지 않는다) ---
        self.steps: list = []        # 이번 사이클에 수행할 단계 목록
        self.step_index = 0
        self.step_elapsed = 0        # 현재 단계에서 소비한 물리 step 수
        self.stable_count = 0        # 목표 근처에서 연속으로 머문 step 수
        self.held_regions: set = set()
        self.standby_active = False  # 잠금 대기로 대기 자세에 물러나 있는 중
        self.box_id = None
        self.task_kind = None       # 현재는 "cycle"만 사용
        self.cycle_number = 0
        self.cycle_steps = 0    # 이번 사이클에 소비한 물리 step (takt 집계용)
        # 이번 사이클에 관절별로 관측한 최대 속도/토크 (제원 대비 사용률 집계용).
        self.peak_joint_speed = np.zeros(len(URDF_JOINT_VELOCITY_LIMITS))
        self.peak_joint_effort = np.zeros(len(URDF_JOINT_EFFORT_LIMITS))
        self.peak_joint_acceleration = np.zeros(len(URDF_JOINT_VELOCITY_LIMITS))
        self.previous_joint_velocity = None
        self.joint_effort_square_sum = np.zeros(len(URDF_JOINT_EFFORT_LIMITS))
        self.joint_effort_samples = 0

    def setup_physics(self) -> None:
        """이 로봇의 EE 경로를 확정하고 drive 강성과 VG10 질량을 보정한다."""
        stage = omni.usd.get_context().get_stage()

        if not stage.GetPrimAtPath(self.ee_prim_path).IsValid():
            found = find_prim_path_by_name(self.root_path, "link_6")
            if found is None:
                raise RuntimeError(f"🚨 '{self.ee_prim_path}'을(를) 찾을 수 없습니다.")
            print(f"  [정보] link_6 하드코딩 경로 대신 탐색으로 찾음: {found}")
            self.ee_path = found
        print(f"  [OK] {self.name} EE = {self.ee_path}")

        robot_root = stage.GetPrimAtPath(self.root_path)
        if not robot_root.IsValid():
            raise RuntimeError(f"로봇 루트 Prim을 찾을 수 없습니다: {self.root_path}")

        drive_count = 0
        for prim in Usd.PrimRange(robot_root):
            for drive_type in ("angular", "linear"):
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    stiffness_attr = drive.GetStiffnessAttr()
                    damping_attr = drive.GetDampingAttr()
                    current_stiffness = stiffness_attr.Get()
                    current_damping = damping_attr.Get()
                    if current_stiffness is not None:
                        stiffness_attr.Set(float(np.clip(current_stiffness, DRIVE_MIN_STIFFNESS, DRIVE_MAX_STIFFNESS)))
                    if current_damping is not None:
                        damping_attr.Set(float(np.clip(current_damping, DRIVE_MIN_DAMPING, DRIVE_MAX_DAMPING)))
                    drive_count += 1
        print(f"  [OK] {self.name} drive {drive_count}개 클램프 적용")

        vg10_prim = stage.GetPrimAtPath(self.vg10_path)
        if not vg10_prim.IsValid():
            found = find_prim_path_by_name(self.root_path.rsplit("/", 1)[0], "VG10_tool")
            if found is not None:
                self.vg10_path = found
                vg10_prim = stage.GetPrimAtPath(found)
        if vg10_prim.IsValid():
            mass_api = UsdPhysics.MassAPI.Apply(vg10_prim)
            mass_api.CreateMassAttr(VG10_MASS_KG)
            print(f"  [OK] {self.name} VG10 질량 {VG10_MASS_KG} kg 보정")

    def add_to_world(self, world: World) -> None:
        self.robot = world.scene.add(
            SingleArticulation(prim_path=self.root_path, name=self.name)
        )
        self.attachment = KinematicAttachment(self.ee_path)

    def initialize(self) -> None:
        """articulation을 초기화하고 이 로봇 고유의 HOME 자세를 적용한다."""
        self.robot.initialize()
        self.robot.set_joint_positions(self.home_pose)

    def create_controller(self) -> None:
        # 서술 파일의 default_q는 이 로봇의 HOME이어야 한다. 두 대의 HOME이
        # joint_1에서 π만큼 다르므로 로봇마다 파일을 따로 만든다.
        description_path = make_description_for_home(
            H2017_DESCRIPTION_PATH,
            self.home_pose,
            Path(GENERATED_RMPFLOW_DIR) / f"{self.name}_description.yaml",
        )
        self.controller = RMPFlowController(
            name=f"{self.name}_rmpflow_controller",
            robot_articulation=self.robot,
            physics_dt=PHYSICS_DT,
            urdf_path=H2017_URDF_PATH,
            robot_description_path=description_path,
            rmpflow_config_path=H2017_RMPFLOW_CONFIG_PATH,
            end_effector_frame_name=EE_FRAME_NAME,
        )
        # physics 재생 중 USD XformCache의 link transform은 뒤처질 수 있다.
        # 부착 박스도 도달 판정과 같은 RMPFlow FK 자세를 사용하게 한다.
        self.attachment.pose_provider = self.controller.get_end_effector_pose

    def measure_gripper_contact_offset(self) -> None:
        """HOME 자세에서 link_6 원점과 VG10 실제 최저면 사이 거리를 측정한다."""
        ee_world_z = float(get_world_position(self.ee_path)[2])
        vg10_aabb = get_world_aabb(self.vg10_path)
        measured = ee_world_z - float(vg10_aabb[2])
        if not np.isfinite(measured) or measured <= 0.0:
            measured = GRIPPER_TIP_OFFSET_M
            print(
                f"[경고] {self.name} VG10 접촉면 자동 측정 실패 — fallback "
                f"{measured:.5f} m 사용"
            )
        self.gripper_contact_offset = measured + GRIPPER_CONTACT_CLEARANCE_M
        print(
            f"[VG10 접촉 계산] {self.name}: link_6→최저면={measured:.5f} m, "
            f"EE offset={self.gripper_contact_offset:.5f} m"
        )

    def capture_home_ee_pose(self) -> None:
        """HOME_POSE가 적용된 현재 EE 자세를 RMPFlow FK 기준으로 저장한다.

        이후 모든 이동에서 같은 자세를 사용하므로 첫 동작 때 6축이 갑자기
        정렬되는 현상을 막는다. 로봇마다 HOME 자세가 다르므로 값도 서로 다르다.
        """
        position, orientation = self.controller.get_end_effector_pose()
        self.home_ee_position = np.asarray(position, dtype=np.float64).copy()
        self.home_ee_orientation = orientation
        print(f"[HOME EE 자세] {self.name}: {np.round(orientation, 5)}")

    def can_reach(self, point: np.ndarray) -> bool:
        return bool(
            np.linalg.norm(np.asarray(point, dtype=np.float64) - self.base_pos)
            <= ROBOT_REACH_LIMIT_M
        )

    # ------------------------------------------------------------------
    # 단계 실행 (두 대가 같은 물리 루프에서 동시에 진행된다)
    #
    # 한 사이클을 "단계 목록"으로 만들어 두고, 매 물리 step마다 현재 단계를
    # 한 칸씩 밀어 준다. 블로킹 함수가 없으므로 두 로봇이 서로를 기다리지 않고
    # 같은 루프 안에서 나란히 움직인다.
    #
    # 단계 종류:
    #   ("move",   target, tolerance, label)  EE를 target으로. 도달하면 다음.
    #   ("pose", target, pos_tol, orientation, angle_tol, label, stable)
    #   ("settle", steps, label)              그 자리에서 steps번 물리 step.
    #   ("servo", target, update, status, label, stable) 실제 payload pose 피드백.
    #   ("call",   fn, label)                 즉시 실행(grab/release 등).
    #   ("lock",   region, label)             구역 점유. 못 잡으면 대기 자세로.
    #   ("unlock", region, label)             구역 반납.
    #   ("gate",   fn, label)                 fn()이 True가 될 때까지 대기.
    # ------------------------------------------------------------------

    @property
    def busy(self) -> bool:
        return self.step_index < len(self.steps)

    def begin_cycle(self, steps, box_id, cycle_number) -> None:
        self.steps = steps
        self.step_index = 0
        self.step_elapsed = 0
        self.stable_count = 0
        self.standby_active = False
        self.box_id = box_id
        self.task_kind = "cycle"
        self.cycle_number = cycle_number
        # takt는 wall-clock이 아니라 물리 step으로 집계한다(명세서 §8).
        self.cycle_steps = 0
        self.peak_joint_speed[:] = 0.0
        self.peak_joint_effort[:] = 0.0
        self.peak_joint_acceleration[:] = 0.0
        self.previous_joint_velocity = None
        self.joint_effort_square_sum[:] = 0.0
        self.joint_effort_samples = 0
        print(f"[CYCLE 시작] {self.name} cycle {cycle_number} ({box_id})")

    def abort_cycle(self, lock: RegionLock) -> None:
        """Timeline 재시작 등으로 진행 중이던 사이클을 폐기한다."""
        self.steps = []
        self.step_index = 0
        self.step_elapsed = 0
        self.stable_count = 0
        self.standby_active = False
        self.box_id = None
        self.task_kind = None
        lock.release_all(self.name)
        self.held_regions.clear()

    def _current(self):
        return self.steps[self.step_index]

    def _track_joint_speed(self) -> None:
        """이번 사이클에서 각 관절이 낸 최대 속도를 기록한다.

        "지금이 H2017의 한계인가"에 답하려면 실제로 관절을 몇 %나 쓰는지 봐야
        한다. RMPFlow의 속도 캡은 스칼라 하나(3.927)라 joint_1~3의 실제 한계를
        지켜 주지 않으므로, 캡 값만 봐서는 알 수 없다.
        """
        velocities = self.robot.get_joint_velocities()
        if velocities is None:
            return
        velocities = np.asarray(velocities, dtype=np.float64)
        if not np.all(np.isfinite(velocities)):
            return
        count = min(len(velocities), len(self.peak_joint_speed))
        if self.previous_joint_velocity is not None:
            acceleration = np.abs(
                (velocities[:count] - self.previous_joint_velocity[:count])
                / PHYSICS_DT
            )
            np.maximum(
                self.peak_joint_acceleration[:count],
                acceleration,
                out=self.peak_joint_acceleration[:count],
            )
        self.previous_joint_velocity = velocities.copy()
        np.maximum(
            self.peak_joint_speed[:count],
            np.abs(velocities[:count]),
            out=self.peak_joint_speed[:count],
        )
        self._track_joint_effort()

    def _track_joint_effort(self) -> None:
        """이번 사이클에서 각 관절이 낸 최대 토크를 기록한다.

        속도가 제원 안이어도 토크가 넘으면 실기에서는 폴트다. 이 API가 없는
        Isaac 버전도 있으므로 없으면 조용히 건너뛴다 — 토크 계측이 없다고 해서
        실행이 죽을 이유는 없다.
        """
        getter = getattr(self.robot, "get_measured_joint_efforts", None)
        if getter is None:
            return
        try:
            efforts = getter()
        except Exception:
            # 이 articulation에서 지원하지 않는 것이므로 다음 step부터 아예 끈다.
            self.robot.get_measured_joint_efforts = None
            return
        if efforts is None:
            return
        efforts = np.asarray(efforts, dtype=np.float64)
        if not np.all(np.isfinite(efforts)):
            return
        count = min(len(efforts), len(self.peak_joint_effort))
        self.joint_effort_square_sum[:count] += np.square(efforts[:count])
        self.joint_effort_samples += 1
        np.maximum(
            self.peak_joint_effort[:count],
            np.abs(efforts[:count]),
            out=self.peak_joint_effort[:count],
        )

    def joint_speed_report(self) -> str:
        """최대 관절 속도와 토크를 URDF 제원 대비 %로 요약한다.

        100%를 넘으면 실기 H2017로는 낼 수 없는 동작이라는 뜻이므로 눈에 띄게
        표시한다. 시뮬레이터는 이 한계를 강제하지 않아, 찍지 않으면 넘긴 줄도
        모른 채 택트만 좋아 보인다.
        """
        parts = []
        speed = self._limit_usage(
            self.peak_joint_speed, URDF_JOINT_VELOCITY_LIMITS, "rad/s"
        )
        if speed:
            parts.append(f"관절속도 {speed}")
        if np.any(self.peak_joint_acceleration):
            worst_accel = int(np.argmax(self.peak_joint_acceleration))
            parts.append(
                f"가속도 peak {self.peak_joint_acceleration[worst_accel]:.2f} "
                f"rad/s² (joint_{worst_accel + 1})"
            )
        effort = self._limit_usage(
            self.peak_joint_effort, URDF_JOINT_EFFORT_LIMITS, "Nm"
        )
        if effort:
            parts.append(f"토크 {effort}")
        if self.joint_effort_samples:
            rms = np.sqrt(
                self.joint_effort_square_sum / self.joint_effort_samples
            )
            worst_rms = int(np.argmax(rms))
            parts.append(
                f"토크 RMS {rms[worst_rms]:.2f} Nm (joint_{worst_rms + 1})"
            )
        return (", " + ", ".join(parts)) if parts else ""

    @staticmethod
    def _limit_usage(peaks, limits, unit: str) -> str:
        count = min(len(peaks), len(limits))
        if count == 0 or not np.any(peaks[:count]):
            return ""
        ratio = peaks[:count] / limits[:count]
        worst = int(np.argmax(ratio))
        flag = " ⚠제원초과" if ratio.max() > 1.0 else ""
        return (
            f"{ratio.max() * 100:.0f}% (joint_{worst + 1}, "
            f"{peaks[worst]:.2f}/{limits[worst]:.2f} {unit}){flag}"
        )

    def _advance(self) -> None:
        self.step_index += 1
        self.step_elapsed = 0
        self.stable_count = 0

    def _drive_to(self, target, orientation) -> None:
        """이번 물리 step에서 EE를 target 쪽으로 미는 관절 명령을 적용한다."""
        joint_positions = self.robot.get_joint_positions()
        if joint_positions is None or not np.all(np.isfinite(joint_positions)):
            raise RuntimeError(
                f"{self.name} 관절 상태가 유효하지 않습니다: {joint_positions}"
            )
        action = self.controller.forward(
            target_end_effector_position=target,
            target_end_effector_orientation=orientation,
        )
        self.robot.apply_action(action)

    def apply_action(self, lock: RegionLock) -> None:
        """물리 step 직전 — 이번 step에 적용할 관절 명령을 계산해 넣는다.

        잠금을 못 얻은 단계에서는 대기 자세(HOME EE)로 물러나는 명령을 넣어,
        멈춰 선 채 상대의 경로를 막지 않게 한다.
        """
        if not self.busy:
            return

        kind = self._current()[0]

        if kind == "move":
            # move는 5번째 안정 스텝과 6번째 목표 자세를 선택적으로 갖는다.
            _, target, _tolerance, _label = self._current()[:4]
            orientation = (
                self._current()[5]
                if len(self._current()) > 5 and self._current()[5] is not None
                else self.home_ee_orientation
            )
            # 베이스 pose 갱신은 동작에 들어갈 때 한 번만 한다. 매 step 갱신하면
            # physics 중 흔들리는 베이스 읽기가 그대로 목표 좌표계에 실려 온다.
            if self.step_elapsed == 0:
                self.controller.update_robot_base_pose()
            self._drive_to(target, orientation)
            return

        if kind == "pose":
            _, target, _pos_tol, orientation, _angle_tol, _label, _stable = (
                self._current()
            )
            if self.step_elapsed == 0:
                self.controller.update_robot_base_pose()
            self._drive_to(target, orientation)
            return

        if kind == "servo":
            _, target, update_target, _status, _label, _stable = self._current()[:6]
            orientation = (
                self._current()[6]
                if len(self._current()) > 6 and self._current()[6] is not None
                else self.home_ee_orientation
            )
            update_target()
            self._drive_to(target, orientation)
            return

        if kind == "lock":
            _, region, _label = self._current()[:3]
            if lock.acquire(region, self.name):
                return  # 이번 step은 명령 없이 넘어가고 observe()에서 진행
            # 중앙 인터락처럼 안전한 사전 staging이 지정된 잠금은 그 위치를
            # 유지한다. 기존 3요소 잠금은 HOME EE로 물러나는 동작을 보존한다.
            wait_target = (
                self._current()[3]
                if len(self._current()) > 3
                else self.home_ee_position
            )
            wait_orientation = (
                self._current()[4]
                if len(self._current()) > 4
                else self.home_ee_orientation
            )
            if not self.standby_active:
                wait_label = "interlock staging 유지" if len(self._current()) > 3 else "대기 자세로 이동"
                print(f"[대기] {self.name}: {region} 점유 중 — {wait_label}")
                self.standby_active = True
                self.controller.update_robot_base_pose()
            self._drive_to(wait_target, wait_orientation)
            return

        if kind == "gate":
            _, condition, _label = self._current()[:3]
            if condition():
                return
            if len(self._current()) > 3 and self._current()[3] == "hold":
                return
            if not self.standby_active:
                print(f"[대기] {self.name}: {self._current()[2]}")
                self.standby_active = True
                self.controller.update_robot_base_pose()
            self._drive_to(self.home_ee_position, self.home_ee_orientation)
            return

        # settle / call / unlock 은 관절 명령이 필요 없다.

    def observe(self, lock: RegionLock) -> None:
        """물리 step 직후 — 도달 여부를 판정하고 다음 단계로 넘긴다."""
        if not self.busy:
            return

        step = self._current()
        kind = step[0]
        self.step_elapsed += 1
        self.cycle_steps += 1

        if kind == "move":
            _, target, tolerance, label = step[:4]
            # 5번째 원소로 이 move만의 안정 스텝을 지정할 수 있다. 없으면 일반값.
            stable_needed = step[4] if len(step) > 4 else STABLE_STEPS
            joint_positions = self.robot.get_joint_positions()
            if joint_positions is None or not np.all(np.isfinite(joint_positions)):
                raise RuntimeError(
                    f"{self.name} {label} 이후 관절 값이 발산했습니다: {joint_positions}"
                )
            self._track_joint_speed()
            current = self.controller.get_end_effector_position()
            if not np.all(np.isfinite(current)):
                raise RuntimeError(f"{self.name} {label} EE pose 무효: {current}")
            error = float(np.linalg.norm(current - target))
            if self.step_elapsed % 180 == 0:
                print(
                    f"  [진행] {self.name} {label}: step={self.step_elapsed}, "
                    f"error={error:.3f} m"
                )
            if error <= tolerance:
                self.stable_count += 1
                if self.stable_count >= stable_needed:
                    # 소요 step을 함께 남긴다. 튜닝 전후를 비교하려면 어느
                    # move가 오래 걸리는지 눈금이 있어야 한다. 안정 확인은
                    # 도달 후의 고정 비용이라 따로 뺀다.
                    moving = self.step_elapsed - stable_needed
                    print(
                        f"[REACHED] {self.name} {label}: error={error:.4f} m, "
                        f"{self.step_elapsed} step ({self.step_elapsed * PHYSICS_DT:.2f}s"
                        f", 이동 {moving} + 안정 {stable_needed})",
                        flush=True,
                    )
                    self._advance()
            else:
                self.stable_count = 0
                if self.step_elapsed >= MAX_STEPS_PER_MOVE:
                    raise RuntimeError(
                        f"{self.name} {label} 도달 시간 초과: "
                        f"current={np.round(current, 3)}, "
                        f"target={np.round(target, 3)}, error={error:.3f} m"
                    )
            return

        if kind == "pose":
            _, target, position_tolerance, target_orientation, angle_tolerance, label, stable_needed = step
            self._track_joint_speed()
            current_position, current_orientation = self.controller.get_end_effector_pose()
            position_error = float(np.linalg.norm(current_position - target))
            angle_error = quaternion_angular_error_deg(
                current_orientation, target_orientation
            )
            if self.step_elapsed % 180 == 0:
                print(
                    f"  [진행] {self.name} {label}: step={self.step_elapsed}, "
                    f"position={position_error:.3f} m, angle={angle_error:.2f}°"
                )
            if position_error <= position_tolerance and angle_error <= angle_tolerance:
                self.stable_count += 1
                if self.stable_count >= stable_needed:
                    print(
                        f"[REACHED] {self.name} {label}: "
                        f"position={position_error:.4f} m, angle={angle_error:.2f}°, "
                        f"{self.step_elapsed} step",
                        flush=True,
                    )
                    self._advance()
            else:
                self.stable_count = 0
                if self.step_elapsed >= MAX_STEPS_PER_MOVE:
                    raise RuntimeError(
                        f"{self.name} {label} 자세 도달 시간 초과: "
                        f"position={position_error:.3f} m, angle={angle_error:.2f}°"
                    )
            return

        if kind == "servo":
            _, _target, _update_target, status, label, stable_needed = step[:6]
            self._track_joint_speed()
            result = status()
            if result.passed:
                self.stable_count += 1
                if self.stable_count >= stable_needed:
                    print(
                        f"[REACHED] {self.name} {label}: "
                        f"gap={result.support_height_error_m * 1000.0:.1f} mm, "
                        f"XY={result.horizontal_drift_m * 1000.0:.1f} mm, "
                        f"tilt={result.tilt_deg:.2f}°, {self.step_elapsed} step",
                        flush=True,
                    )
                    self._advance()
            else:
                self.stable_count = 0
                if self.step_elapsed % 180 == 0:
                    print(
                        f"  [진행] {self.name} {label}: step={self.step_elapsed}, "
                        f"{', '.join(result.reasons)}",
                        flush=True,
                    )
                if self.step_elapsed >= MAX_STEPS_PER_MOVE:
                    raise RuntimeError(
                        f"{self.name} {label} 시간 초과: {', '.join(result.reasons)}"
                    )
            return

        if kind == "settle":
            _, steps, _label = step
            if self.step_elapsed >= steps:
                self._advance()
            return

        if kind == "call":
            _, fn, label = step
            fn()
            print(f"[STEP] {self.name} {label}")
            self._advance()
            return

        if kind == "lock":
            _, region, _label = step[:3]
            if lock.owner[region] == self.name:
                self.held_regions.add(region)
                self.standby_active = False
                self._advance()
            return

        if kind == "unlock":
            _, region, _label = step
            lock.release(region, self.name)
            self.held_regions.discard(region)
            self._advance()
            return

        if kind == "gate":
            _, condition, _label = step[:3]
            if condition():
                self.standby_active = False
                self._advance()
            return

        raise RuntimeError(f"알 수 없는 단계 종류: {kind}")

class KinematicAttachment:
    """gripper_prim_path를 따라가도록 다른 prim을 고정된 상대 변환으로 붙인다."""

    def __init__(self, gripper_prim_path: str) -> None:
        self.gripper_prim_path = gripper_prim_path
        self.pose_provider = None
        self.carried_prim_path = None
        self.relative_matrix = Gf.Matrix4d(1.0)
        self.box_scale = Gf.Vec3d(1.0)
        self.box_matrix_op = None

    @staticmethod
    def _stage():
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("열린 USD Stage가 없습니다.")
        return stage

    @staticmethod
    def _set_kinematic(root_prim, enabled: bool) -> int:
        count = 0
        for prim in Usd.PrimRange(root_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body = UsdPhysics.RigidBodyAPI(prim)
                rigid_body.GetKinematicEnabledAttr().Set(enabled)
                if not enabled:
                    rigid_body.GetVelocityAttr().Set(Gf.Vec3f(0.0))
                    rigid_body.GetAngularVelocityAttr().Set(Gf.Vec3f(0.0))
                count += 1
        return count

    @staticmethod
    def _set_collision(root_prim, enabled: bool) -> int:
        """운반 중 그리퍼와 kinematic 큐브 사이의 충돌력 누적을 막는다."""
        count = 0
        for prim in Usd.PrimRange(root_prim):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            collision = UsdPhysics.CollisionAPI(prim)
            collision.CreateCollisionEnabledAttr(enabled).Set(enabled)
            count += 1
        return count

    @staticmethod
    def _rigid_matrix(matrix):
        """스케일/전단을 버리고 위치와 직교 회전만 남긴다.

        비균일 스케일이 들어간 박스 행렬을 그리퍼 행렬과 그대로 곱하면 회전
        중 scale 축이 섞여 shear가 생길 수 있다. 운반 상대 자세는 rigid
        transform으로만 계산하고, 박스의 고정 scale은 마지막에 한 번 적용한다.
        """
        transform = Gf.Transform(matrix)
        rigid = Gf.Matrix4d(1.0)
        rigid.SetRotate(transform.GetRotation())
        rigid.SetTranslateOnly(transform.GetTranslation())
        return rigid

    def _gripper_rigid_world(self, cache, gripper):
        if self.pose_provider is None:
            return self._rigid_matrix(cache.GetLocalToWorldTransform(gripper))
        position, quaternion = self.pose_provider()
        quaternion = np.asarray(quaternion, dtype=np.float64)
        rigid = Gf.Matrix4d(1.0)
        rigid.SetRotate(
            Gf.Quatd(
                float(quaternion[0]),
                Gf.Vec3d(*map(float, quaternion[1:4])),
            )
        )
        rigid.SetTranslateOnly(Gf.Vec3d(*map(float, position)))
        return rigid

    def grab(self, box_prim_path: str) -> None:
        stage = self._stage()
        gripper = stage.GetPrimAtPath(self.gripper_prim_path)
        box = stage.GetPrimAtPath(box_prim_path)
        if not gripper.IsValid():
            raise RuntimeError(f"그리퍼 추종 Prim이 없습니다: {self.gripper_prim_path}")
        if not box.IsValid():
            raise RuntimeError(f"박스 Prim이 없습니다: {box_prim_path}")

        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        box_world = cache.GetLocalToWorldTransform(box)

        box_transform = Gf.Transform(box_world)
        self.box_scale = Gf.Vec3d(box_transform.GetScale())
        box_rigid_world = self._rigid_matrix(box_world)
        gripper_rigid_world = self._gripper_rigid_world(cache, gripper)
        # USD의 row-vector 변환 관례: desired_box_world = relative * gripper_world
        self.relative_matrix = box_rigid_world * gripper_rigid_world.GetInverse()
        rigid_body_count = self._set_kinematic(box, True)
        if rigid_body_count == 0:
            raise RuntimeError(f"박스에 RigidBodyAPI가 없습니다: {box_prim_path}")
        collision_count = self._set_collision(box, False)

        self.box_matrix_op = UsdGeom.Xformable(box).MakeMatrixXform()
        self.carried_prim_path = box_prim_path
        self.update()
        print(
            f"[GRAB] {box_prim_path} (rigid bodies={rigid_body_count}, "
            f"운반 중 collision OFF={collision_count})"
        )

    def update(self) -> None:
        if self.carried_prim_path is None or self.box_matrix_op is None:
            return

        stage = self._stage()
        gripper = stage.GetPrimAtPath(self.gripper_prim_path)
        box = stage.GetPrimAtPath(self.carried_prim_path)
        if not gripper.IsValid() or not box.IsValid():
            raise RuntimeError("운반 중인 그리퍼 또는 박스 Prim이 사라졌습니다.")

        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        gripper_rigid_world = self._gripper_rigid_world(cache, gripper)
        desired_box_rigid_world = self.relative_matrix * gripper_rigid_world
        scale_matrix = Gf.Matrix4d(1.0)
        scale_matrix.SetScale(self.box_scale)
        desired_box_world = scale_matrix * desired_box_rigid_world

        parent = box.GetParent()
        if parent.IsValid() and not parent.IsPseudoRoot():
            parent_world = cache.GetLocalToWorldTransform(parent)
            desired_box_local = desired_box_world * parent_world.GetInverse()
        else:
            desired_box_local = desired_box_world

        self.box_matrix_op.Set(desired_box_local)

    def release(self) -> None:
        if self.carried_prim_path is None:
            print("[RELEASE] 운반 중인 박스가 없습니다.")
            return

        stage = self._stage()
        box_path = self.carried_prim_path
        box = stage.GetPrimAtPath(box_path)

        self.carried_prim_path = None
        self.box_matrix_op = None

        rigid_body_count = self._set_kinematic(box, False) if box.IsValid() else 0
        collision_count = self._set_collision(box, True) if box.IsValid() else 0
        print(
            f"[RELEASE] {box_path} (dynamic rigid bodies={rigid_body_count}, "
            f"collision ON={collision_count})"
        )

    def clear_after_timeline_stop(self) -> None:
        """Stage가 초기 상태로 복구된 뒤 오래된 grab 상태만 폐기한다."""
        self.carried_prim_path = None
        self.box_matrix_op = None
        self.relative_matrix = Gf.Matrix4d(1.0)
        self.box_scale = Gf.Vec3d(1.0)
