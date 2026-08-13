
"""Conveyor speed control, timed box spawning, and pick-zone gating."""

from __future__ import annotations

import numpy as np
import omni.usd

from .config import *
from .coordination import has_reached_pick_zone
from .scene import get_world_position


class ConveyorSpeedController:
    """USD 진행 방향을 유지하며 속도를 0/설정 속력으로 전환한다."""

    def __init__(self, attribute_paths, name: str) -> None:
        self.attribute_paths = tuple(attribute_paths)
        self.name = str(name)
        self.attributes = []
        self.running_values = []
        # 발견 시 USD 속력을 런타임 설정값으로 덮고 start()/stop()이 전환한다.
        self._running = True
        self._discover_attributes()

    @staticmethod
    def _is_numeric_scalar(value) -> bool:
        return isinstance(value, (int, float, np.integer, np.floating)) and not isinstance(value, bool)

    def _discover_attributes(self) -> None:
        """CONVEYOR_SPEED_ATTRIBUTE_PATHS의 속성만 제어 대상으로 잡는다.

        이름으로 자동 탐색하지 않는다. 예전에는 "prim 경로에 conveyor,
        속성명에 velocity"로 폴백했는데, 그 규칙이
        카메라의 RSD455/Imu_Sensor.angularVelocityFilterWidth를
        컨베이어로 오인해 stop()마다 0으로 만들었다. 경로가 틀리면 조용히
        엉뚱한 걸 잡는 것보다 시작하자마자 죽는 편이 낫다.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("컨베이어 속성 탐색 중 열린 USD Stage가 없습니다.")

        for attr_path in self.attribute_paths:
            attr = stage.GetAttributeAtPath(attr_path)
            value = attr.Get() if attr and attr.IsValid() else None
            if not self._is_numeric_scalar(value):
                raise RuntimeError(
                    f"{self.name} 속도 Attribute가 유효하지 않습니다: {attr_path} "
                    f"(값={value!r}). USD에서 실제 경로를 확인해 "
                    f"CONVEYOR_SPEED_ATTRIBUTE_PATHS를 고치세요."
                )
            if float(value) == 0.0:
                raise RuntimeError(
                    f"{self.name} USD 속도가 0이라 진행 방향을 알 수 없습니다: "
                    f"{attr_path}"
                )
            running_value = float(np.copysign(CONVEYOR_RUN_SPEED_MPS, value))
            attr.Set(running_value)
            self.attributes.append(attr)
            self.running_values.append(running_value)

        print(f"[CONVEYOR] {self.name} 제어 대상 속성:")
        for attr, value in zip(self.attributes, self.running_values):
            print(
                f"  {attr.GetPath()} = {value:.3f} "
                f"(설정 속력 {CONVEYOR_RUN_SPEED_MPS:.3f} m/s)"
            )

    def is_running(self) -> bool:
        return self._running

    def stop(self) -> None:
        # 매 물리 step 콜백에서 호출되므로 상태가 바뀔 때만 쓰고 로그도 남긴다.
        if not self._running:
            return
        for attr in self.attributes:
            attr.Set(0.0)
        self._running = False
        print(f"[CONVEYOR] {self.name} STOP")

    def start(self) -> None:
        if self._running:
            return
        for attr, value in zip(self.attributes, self.running_values):
            attr.Set(value)
        self._running = True
        print(f"[CONVEYOR] {self.name} START")

class BoxSpawner:
    """계획 순서대로 박스를 "시간 간격"에 맞춰 컨베이어에 투입한다.

    로봇이 HOME에 복귀했는지와 무관하게 물리 시간만 보고 스폰하므로, 로봇
    사이클보다 간격이 짧으면 박스가 컨베이어 위에 줄을 서고 로봇은 대기 없이
    연속으로 일한다. 반대로 로봇이 빠르면 다음 박스가 올 때까지 기다린다.

    같은 자리에 겹쳐 생성하면 PhysX가 두 강체를 밀어내며 폭주하므로, 직전
    박스가 생성 지점을 벗어나기 전에는 스폰을 미룬다. 건너뛰는 게 아니라
    미루는 것이라서 총 개수(--box-count)는 그대로 지켜진다.
    """

    def __init__(
        self,
        spawn_box,
        total_count: int,
        interval_sec: float,
        spawn_position,
        clearance_m: float = BOX_SPAWN_CLEARANCE_M,
        name: str = "conveyor",
    ) -> None:
        self.spawn_box = spawn_box          # plan_index -> cube_path
        self.total_count = total_count
        self.interval_sec = interval_sec
        self.spawn_position = np.asarray(spawn_position, dtype=np.float64).reshape(3)
        self.clearance_m = clearance_m
        self.name = str(name)
        self.reset()

    def reset(self) -> None:
        """Timeline Stop/Play로 장면이 초기화됐을 때 타이머까지 되돌린다."""
        self.elapsed_sec = 0.0
        self.next_spawn_sec = 0.0
        self.spawned_count = 0
        # 아직 로봇이 집어가지 않은 박스들. 앞쪽이 먼저 투입된(=먼저 도착할) 것.
        self.pending: list[str] = []
        self.last_spawn_path = None
        self.delay_logged = False

    @property
    def finished(self) -> bool:
        return self.spawned_count >= self.total_count

    def prime(self) -> str | None:
        """타이머를 초기화하고 첫 박스를 즉시 투입한다.

        첫 박스는 world.reset() 전에 존재해야 PhysX가 초기 장면 구성 때부터
        인식하므로 타이머를 기다리지 않는다. 두 번째부터 간격이 적용된다.
        """
        self.reset()
        if self.total_count <= 0:
            return None
        path = self.spawn_box(0)
        self.pending.append(path)
        self.last_spawn_path = path
        self.spawned_count = 1
        self.next_spawn_sec = self.interval_sec
        return path

    def _spawn_area_clear(self) -> bool:
        if self.last_spawn_path is None:
            return True
        try:
            position = get_world_position(self.last_spawn_path)
        except RuntimeError:
            # 직전 박스가 이미 사라졌으면(집혀서 팔레트로 감) 생성 지점은 비었다.
            return True
        return bool(
            np.linalg.norm(
                np.asarray(position, dtype=np.float64)[:2]
                - self.spawn_position[:2]
            ) >= self.clearance_m
        )

    def tick(self, dt: float) -> None:
        """물리 1 step만큼 시간을 진행시키고, 때가 되면 박스를 투입한다."""
        self.elapsed_sec += dt
        if self.finished or self.elapsed_sec < self.next_spawn_sec:
            return

        if not self._spawn_area_clear():
            if not self.delay_logged:
                print(
                    f"[SPAWN 지연 {self.name}] 생성 지점에 직전 박스가 남아 있어 대기 "
                    f"(t={self.elapsed_sec:.1f}s)"
                )
                self.delay_logged = True
            return

        path = self.spawn_box(self.spawned_count)
        self.pending.append(path)
        self.last_spawn_path = path
        self.spawned_count += 1
        self.next_spawn_sec = self.elapsed_sec + self.interval_sec
        self.delay_logged = False
        print(
            f"[SPAWN 타이머 {self.name}] t={self.elapsed_sec:.1f}s "
            f"({self.spawned_count}/{self.total_count}) 대기열={len(self.pending)}"
        )

    def front(self):
        """컨베이어에서 가장 먼저 도착할 박스 경로 (없으면 None)."""
        return self.pending[0] if self.pending else None

    def pop_front(self) -> str | None:
        return self.pending.pop(0) if self.pending else None

class ConveyorGate:
    """Stop the conveyor when the queue's front box reaches the pick zone."""

    def __init__(
        self,
        conveyor,
        spawner,
        pick_zone_min,
        pick_zone_max,
        name: str,
    ) -> None:
        self.conveyor = conveyor
        self.spawner = spawner
        self.pick_zone_min = np.asarray(pick_zone_min, dtype=np.float64).reshape(3)
        self.pick_zone_max = np.asarray(pick_zone_max, dtype=np.float64).reshape(3)
        self.name = str(name)
        if np.any(self.pick_zone_min > self.pick_zone_max):
            raise ValueError(
                f"{self.name} Pick Zone 범위 오류: "
                f"{self.pick_zone_min} > {self.pick_zone_max}"
            )

    def contains(self, position) -> bool:
        return has_reached_pick_zone(
            position,
            self.pick_zone_min,
            self.pick_zone_max,
        )

    def update(self) -> None:
        if not self.conveyor.is_running():
            return
        front_path = self.spawner.front()
        if front_path is None:
            return
        try:
            position = get_world_position(front_path)
        except RuntimeError:
            return
        if not self.contains(position):
            return

        # 라인별 비전 트리거는 쏘지 않는다. 측정 패스가 끝난 뒤라
        # 아무도 검출 결과를 읽지 않는데 박스마다 포인트클라우드 연산만 돈다.
        # 측정 트리거는 IntakeStation이 안정화가 끝난 뒤 직접 발행한다.
        self.conveyor.stop()
        print(
            f"[PICK ZONE {self.name}] 도착 — 컨베이어 정지: {front_path} "
            f"{np.round(position, 3)}"
        )
