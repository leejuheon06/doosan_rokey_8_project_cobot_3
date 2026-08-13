from pathlib import Path

import isaacsim.robot_motion.motion_generation as mg
import numpy as np
import omni.usd
from isaacsim.core.prims import SingleArticulation
from pxr import UsdGeom
from scipy.spatial.transform import Rotation


def make_description_for_home(
    base_description_path: str,
    home_pose,
    output_path: str | Path,
) -> str:
    """HOME 자세를 default_q로 갖는 로봇 서술 파일을 만들어 그 경로를 돌려준다.

    RMPflow의 cspace_target_rmp(metric 50, position gain 100)는 매 step 자세를
    default_q 쪽으로 당긴다. 이 장면의 두 로봇은 컨베이어를 사이에 두고 서로
    반대쪽을 보므로 joint_1이 각각 -π/2와 +π/2인데, 원본 서술 파일의
    default_q는 단일 로봇 기준인 joint_1=0이다. 그대로 쓰면 두 대 모두 작업
    자세와 반대 방향으로 끌려 수렴이 느려지고 평형 오차가 커진다.

    나머지 항목(가속/저크 한계, 충돌 구)은 로봇이 같으므로 건드리지 않는다.
    주석까지 보존하려고 yaml 파싱 대신 default_q 줄만 바꿔 쓴다.
    """
    home_pose = [float(value) for value in home_pose]
    lines = Path(base_description_path).read_text(encoding="utf-8").splitlines()
    replaced = 0
    for index, line in enumerate(lines):
        if line.startswith("default_q:"):
            lines[index] = "default_q: [" + ", ".join(f"{v:.4f}" for v in home_pose) + "]"
            replaced += 1
    if replaced != 1:
        raise RuntimeError(
            f"{base_description_path}에서 default_q 줄을 정확히 하나 찾지 못했습니다 "
            f"(발견 {replaced}개)."
        )

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[RMPFlow] default_q={np.round(home_pose, 4)} 서술 파일 생성: {output_path}")
    return str(output_path)


class RMPFlowController(mg.MotionPolicyController):
    """H2017용 RMPFlow controller."""

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        urdf_path: str | None = None,
        robot_description_path: str | None = None,
        rmpflow_config_path: str | None = None,
        end_effector_frame_name: str = "link_6",
        maximum_substep_size: float = 0.00334,
    ) -> None:
        base_dir = Path(__file__).resolve().parent
        project_dir = base_dir.parent
        urdf_path = str(Path(urdf_path) if urdf_path else project_dir / "h2017.urdf")
        robot_description_path = str(
            Path(robot_description_path) if robot_description_path else base_dir / "h2017_description.yaml"
        )
        rmpflow_config_path = str(
            Path(rmpflow_config_path) if rmpflow_config_path else base_dir / "h2017_rmpflow_common.yaml"
        )

        self.rmp_flow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            urdf_path=urdf_path,
            end_effector_frame_name=end_effector_frame_name,
            maximum_substep_size=maximum_substep_size,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        self._robot_articulation = robot_articulation
        self._base_pose_warning_printed = False
        self._default_position, self._default_orientation = self._get_valid_base_pose()
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )

    def _get_valid_base_pose(self):
        position, orientation = self._robot_articulation.get_world_pose()
        position = np.asarray(position, dtype=float)
        orientation = np.asarray(orientation, dtype=float)

        orientation_norm = np.linalg.norm(orientation)
        pose_is_valid = (
            np.all(np.isfinite(position))
            and np.all(np.isfinite(orientation))
            and orientation_norm >= 1e-8
        )

        if not pose_is_valid:
            position, orientation = self._get_usd_base_pose()
            if not self._base_pose_warning_printed:
                print(
                    "[RMPFlow 경고] Articulation 베이스 pose가 유효하지 않아 "
                    "USD 월드 transform을 사용합니다."
                )
                self._base_pose_warning_printed = True
        else:
            orientation = orientation / orientation_norm

        return position, orientation

    def _get_usd_base_pose(self):
        stage = omni.usd.get_context().get_stage()
        base_prim = stage.GetPrimAtPath(self._robot_articulation.prim_path)
        if not base_prim.IsValid():
            raise RuntimeError(
                f"로봇 베이스 Prim을 찾을 수 없습니다: {self._robot_articulation.prim_path}"
            )

        transform = UsdGeom.XformCache().GetLocalToWorldTransform(base_prim)
        translation = transform.ExtractTranslation()
        rotation = transform.ExtractRotationQuat()
        imaginary = rotation.GetImaginary()

        position = np.array(
            [translation[0], translation[1], translation[2]], dtype=float
        )
        orientation = np.array(
            [rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]],
            dtype=float,
        )

        orientation_norm = np.linalg.norm(orientation)
        if (
            not np.all(np.isfinite(position))
            or not np.all(np.isfinite(orientation))
            or orientation_norm < 1e-8
        ):
            raise RuntimeError(
                f"USD에서도 유효한 로봇 베이스 pose를 얻지 못했습니다: "
                f"position={position}, orientation={orientation}"
            )

        return position, orientation / orientation_norm

    def update_robot_base_pose(self):
        position, orientation = self._get_valid_base_pose()
        self._default_position = position
        self._default_orientation = orientation
        self._motion_policy.set_robot_base_pose(
            robot_position=position,
            robot_orientation=orientation,
        )

    def get_end_effector_pose(self):
        """Return the EE pose from the same model/state used by RMPflow.

        Reading an articulation link with a USD XformCache while physics is
        running can return a stale transform.  Convergence checks must instead
        use RMPflow's active joint state and forward kinematics.
        """
        if not self._robot_articulation.handles_initialized:
            raise RuntimeError(
                "RMPFlow 관절 상태를 읽기 전에 Articulation handle이 해제되었습니다."
            )
        joint_positions = (
            self.articulation_rmp
            .get_active_joints_subset()
            .get_joint_positions()
        )
        if joint_positions is None:
            raise RuntimeError("RMPFlow 활성 관절 위치를 읽을 수 없습니다.")
        position, rotation_matrix = self.rmp_flow.get_end_effector_pose(
            np.asarray(joint_positions, dtype=float)
        )
        quaternion_xyzw = Rotation.from_matrix(rotation_matrix).as_quat()
        quaternion_wxyz = quaternion_xyzw[[3, 0, 1, 2]]
        return (
            np.asarray(position, dtype=float),
            np.asarray(quaternion_wxyz, dtype=float),
        )

    def get_end_effector_position(self):
        return self.get_end_effector_pose()[0]

    def reset(self):
        super().reset()
        self.update_robot_base_pose()
