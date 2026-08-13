"""Pick Zone에 선 박스를 비전으로 재는 온라인 측정 단계.

박스를 한 개만 물리 scene에 두고, 잰 그 박스를 그대로 적재한다. 전수 측정 후
일괄 계획하던 2패스는 없앴다.

Isaac에 의존하지 않는 순수 모듈이라 pytest로 바로 돌아간다.
"""

from __future__ import annotations

from dataclasses import dataclass

from .catalog import is_empty_detection, parse_detection
from .config import (
    MEASUREMENT_EMPTY_RETRY_COUNT,
    MEASUREMENT_SETTLE_STEPS,
    MEASUREMENT_TIMEOUT_SEC,
    PHYSICS_DT,
)


class MeasurementPassError(RuntimeError):
    """측정을 계속할 수 없을 때. 조용히 넘기면 원인 찾기가 괴로워진다."""


@dataclass
class Measurement:
    box_id: str
    number: int                                  # 비전이 스냅한 호수
    dimensions: tuple[float, float, float]       # 그 호수의 공칭 치수
    truth_number: int                            # 실제로 스폰한 호수 (정답)

    @property
    def correct(self) -> bool:
        return self.number == self.truth_number


class IntakeStation:
    """Pick Zone에 선 박스 하나를 재는 논블로킹 상태기계.

    물리 루프를 붙잡지 않는다. 한쪽 로봇이 적재하는 동안 다른 쪽이 멈추면 안
    되기 때문이다. 메인 루프가 매 step update()를 부르고 state를 본다.

    벨트가 서고 -> settle_steps 만큼 흔들림이 잦아들기를 기다렸다가 -> 트리거를
    쏘고 -> 결과를 기다린다. 흔들리는 동안 재면 치수가 튄다.
    """

    WAITING = "waiting"
    READY = "ready"
    FAILED = "failed"

    def __init__(
        self,
        conveyor,
        ros,
        settle_steps: int = MEASUREMENT_SETTLE_STEPS,
        timeout_steps: int = int(MEASUREMENT_TIMEOUT_SEC / PHYSICS_DT),
        empty_retry_count: int = MEASUREMENT_EMPTY_RETRY_COUNT,
    ) -> None:
        self.conveyor = conveyor
        self.ros = ros
        self.settle_steps = settle_steps
        self.timeout_steps = timeout_steps
        self.empty_retry_count = max(0, int(empty_retry_count))
        self.path: str | None = None
        self.reset()

    def reset(self, path: str | None = None) -> None:
        self.path = path
        self.state = self.WAITING
        self.steps = 0
        self.number: int | None = None
        self.dimensions: tuple[float, float, float] | None = None
        self.timed_out = False
        self.trigger_sent = False
        self.empty_retries = 0

    def update(self, front_path: str | None, at_pick_zone: bool) -> None:
        """물리 1 step만큼 상태를 진행한다.

        at_pick_zone은 "앞 박스가 Pick Zone 안에 들어와 있는가"다. 벨트 정지
        여부만으로는 안 된다 — 로봇이 박스를 집는 동안에도 벨트는 서기 때문에,
        그때 재면 아직 한참 위에 있는 다음 박스나 시야에 들어온 로봇 팔을
        재고 만다(실측: 스폰 지점 x=-2.159의 박스를 측정해 버림).
        """
        if front_path != self.path:
            self.reset(front_path)
        if front_path is None or self.state != self.WAITING:
            return
        if not at_pick_zone or self.conveyor.is_running():
            # 아직 Pick Zone에 서지 않았다. 게이트가 세워 주기를 기다린다.
            self.steps = 0
            return

        self.steps += 1

        # settle_steps=0도 첫 eligible update에서 즉시 트리거해야 한다. 기존의
        # ``steps <= settle_steps`` 조건은 0일 때 한 번도 참이 되지 않아 비전
        # 요청 자체가 나가지 않았다. 최소 1회 update는 이전 결과를 비우는 데
        # 사용하되, 0과 1은 모두 명시적 안정화 대기 없이 즉시 요청한다.
        trigger_step = max(1, self.settle_steps)
        if self.steps <= trigger_step:
            # 안정화 내내 큐를 비운다. spin_once(timeout_sec=0)는 콜백이 아직
            # 도착하지 않았으면 아무것도 처리하지 않으므로, 트리거 직전 1회로는
            # 직전 박스의 결과가 DDS 큐에 남는다(실측: 6번째 박스가 5번째의
            # 결과를 받아 오분류).
            self.ros.poll_detection()
            if self.steps == trigger_step:
                self.ros.publish_conveyor_stopped()
                self.trigger_sent = True
            return

        payload = self.ros.poll_detection()
        if payload is not None:
            snapped = parse_detection(payload)
            if snapped is None:
                if (
                    is_empty_detection(payload)
                    and self.empty_retries < self.empty_retry_count
                ):
                    self.empty_retries += 1
                    self.ros.publish_conveyor_stopped()
                    return
                self.state = self.FAILED
            else:
                self.number, self.dimensions = snapped
                self.state = self.READY
            return

        if self.steps > self.settle_steps + self.timeout_steps:
            self.state = self.FAILED
            self.timed_out = True


def report_accuracy(measurements: list[Measurement], attempted: int) -> None:
    """분류 정확도를 로그로 남긴다. 이것이 sim-in-the-loop 검증의 산출물이다."""
    detected = len(measurements)
    correct = sum(1 for measurement in measurements if measurement.correct)
    print("\n" + "=" * 60, flush=True)
    print("[비전 검증]", flush=True)
    print(f"  투입      {attempted}개", flush=True)
    if attempted:
        print(f"  검출      {detected}개 ({detected / attempted:.1%})", flush=True)
    if detected:
        print(
            f"  분류 정확 {correct}/{detected} ({correct / detected:.1%})", flush=True
        )
        for measurement in measurements:
            if not measurement.correct:
                print(
                    f"    오분류 {measurement.box_id}: "
                    f"{measurement.truth_number}호 → {measurement.number}호",
                    flush=True,
                )
    print("=" * 60 + "\n", flush=True)
