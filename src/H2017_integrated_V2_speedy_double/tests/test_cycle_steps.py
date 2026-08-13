"""사이클 단계 목록의 모양이 robot.py의 언팩과 맞는지 확인한다.

application.build_cycle_steps는 Isaac이 필요해 여기서 부를 수 없다. 대신
robot.py가 각 kind를 어떻게 언팩하는지에 맞춰, 실제로 쓰는 것과 같은 모양의
단계 목록을 만들어 검사한다. move가 4개짜리와 5개짜리(안정 스텝 지정) 두 형태를
모두 갖기 때문에, 한쪽만 지원하는 언팩이 남으면 여기서 걸린다.
"""

import unittest

from h2017_palletizing.config import (
    GRAB_SETTLE_STEPS,
    PLACE_DESCEND_STABLE_STEPS,
    PLACE_RELEASE_SERVO_STABLE_STEPS,
    RELEASE_SETTLE_STEPS,
    STABLE_STEPS,
)


def sample_cycle_steps():
    """application.build_cycle_steps가 내는 것과 같은 모양의 목록."""
    return [
        ("lock", "pick_zone", "Pick Zone 확보"),
        ("move", (0.0, 0.0, 1.0), 0.035, "pick approach"),
        ("move", (0.0, 0.0, 0.8), 0.035, "pick descend"),
        ("call", lambda: None, "grab"),
        ("settle", GRAB_SETTLE_STEPS, "grab settle"),
        ("move", (0.0, 0.0, 1.0), 0.035, "pick lift"),
        ("move", (0.0, 0.55, 1.0), 0.055, "payload rotation staging"),
        ("unlock", "pick_zone", "Pick Zone 반납"),
        ("call", lambda: None, "컨베이어 재시작"),
        ("pose", (0.0, 0.55, 1.0), 0.055, (0.707, 0.0, 0.0, 0.707),
         3.0, "payload yaw align", 10),
        ("move", (0.7, 0.3, 1.0), 0.055, "center interlock staging",
         STABLE_STEPS, (0.707, 0.0, 0.0, 0.707)),
        ("gate", lambda: True, "지지 박스 대기", "hold"),
        ("lock", "pallet_center", "팔레트 중앙 교차구역 확보",
         (0.7, 0.3, 1.0), (0.707, 0.0, 0.0, 0.707)),
        ("move", (1.0, 0.0, 1.0), 0.055, "place approach"),
        ("move", (1.0, 0.0, 0.5), 0.035, "place descend",
         PLACE_DESCEND_STABLE_STEPS),
        ("servo", [1.0, 0.0, 0.5], lambda: None, lambda: None,
         "place release align", PLACE_RELEASE_SERVO_STABLE_STEPS),
        ("call", lambda: None, "release"),
        ("gate", lambda: True, "release settle"),
        ("move", (1.0, 0.0, 1.0), 0.055, "place retreat"),
        ("move", (0.7, 0.3, 1.0), 0.055, "center interlock exit",
         STABLE_STEPS, (0.707, 0.0, 0.0, 0.707)),
        ("unlock", "pallet_center", "팔레트 중앙 교차구역 반납"),
        ("move", (0.0, 0.55, 1.0), 0.055, "post-place side standby"),
    ]


class CycleStepShapeTest(unittest.TestCase):
    def test_every_step_unpacks_the_way_robot_expects(self):
        for step in sample_cycle_steps():
            kind = step[0]
            if kind == "move":
                # robot.py: step[:4] + 선택적 step[4]
                _, target, tolerance, label = step[:4]
                stable = step[4] if len(step) > 4 else STABLE_STEPS
                self.assertEqual(len(target), 3)
                self.assertGreater(tolerance, 0)
                self.assertIsInstance(label, str)
                self.assertGreaterEqual(stable, 1)
            elif kind == "pose":
                _, target, position_tolerance, orientation, angle_tolerance, label, stable = step
                self.assertEqual(len(target), 3)
                self.assertEqual(len(orientation), 4)
                self.assertGreater(position_tolerance, 0)
                self.assertGreater(angle_tolerance, 0)
                self.assertIsInstance(label, str)
                self.assertGreaterEqual(stable, 1)
            elif kind == "settle":
                _, steps, _label = step
                self.assertGreaterEqual(steps, 1)
            elif kind == "servo":
                _, target, update, status, label, stable = step
                self.assertEqual(len(target), 3)
                self.assertTrue(callable(update))
                self.assertTrue(callable(status))
                self.assertIsInstance(label, str)
                self.assertGreaterEqual(stable, 1)
            elif kind == "call":
                _, fn, _label = step
                self.assertTrue(callable(fn))
            elif kind in ("lock", "unlock"):
                _, region, _label = step[:3]
                self.assertIsInstance(region, str)
                if kind == "lock" and len(step) > 3:
                    self.assertEqual(len(step[3]), 3)
                    self.assertEqual(len(step[4]), 4)
            elif kind == "gate":
                _, condition, _label = step[:3]
                self.assertTrue(callable(condition))
            else:
                self.fail(f"robot.py가 모르는 단계 종류: {kind}")

    def test_place_descend_is_the_only_stricter_move(self):
        stricter = [
            step[3] for step in sample_cycle_steps()
            if step[0] == "move" and len(step) > 4 and step[4] != STABLE_STEPS
        ]
        self.assertEqual(stricter, ["place descend"])

    def test_center_lock_is_released_after_exit_and_before_side_standby(self):
        labels = [step[3] if step[0] == "move" else step[2] for step in sample_cycle_steps()]
        self.assertLess(
            labels.index("center interlock exit"),
            labels.index("팔레트 중앙 교차구역 반납"),
        )
        self.assertLess(
            labels.index("팔레트 중앙 교차구역 반납"),
            labels.index("post-place side standby"),
        )

    def test_release_alignment_follows_place_descend(self):
        labels = [step[-2] if step[0] == "servo" else step[3]
                  for step in sample_cycle_steps() if step[0] in ("move", "servo")]
        index = labels.index("place descend")
        self.assertEqual(labels[index:index + 2], ["place descend", "place release align"])

    def test_yaw_is_completed_after_leaving_pick_zone_and_before_place(self):
        steps = sample_cycle_steps()
        labels = [
            step[5] if step[0] == "pose"
            else step[3] if step[0] == "move"
            else step[2]
            for step in steps
        ]
        self.assertLess(labels.index("Pick Zone 반납"), labels.index("payload yaw align"))
        self.assertLess(labels.index("payload yaw align"), labels.index("place approach"))

    def test_cycle_ends_at_the_side_standby(self):
        """Place 뒤에는 벨트 경로를 비우는 측면 위치에서 대기한다."""
        labels = [step[3] for step in sample_cycle_steps() if step[0] == "move"]
        self.assertNotIn("HOME 복귀", labels)
        self.assertEqual(
            labels[-3:],
            ["place retreat", "center interlock exit", "post-place side standby"],
        )

    def test_settle_values_match_stable_reference(self):
        self.assertEqual(GRAB_SETTLE_STEPS, 10)
        self.assertEqual(RELEASE_SETTLE_STEPS, 30)
        self.assertEqual(STABLE_STEPS, 5)
        self.assertEqual(PLACE_DESCEND_STABLE_STEPS, 25)
        self.assertGreater(PLACE_DESCEND_STABLE_STEPS, STABLE_STEPS)


if __name__ == "__main__":
    unittest.main()
