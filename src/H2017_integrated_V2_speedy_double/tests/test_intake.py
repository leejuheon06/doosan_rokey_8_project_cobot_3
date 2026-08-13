import json
import unittest

from h2017_palletizing.intake import IntakeStation


class FakeConveyor:
    def __init__(self, running=False):
        self._running = running

    def is_running(self):
        return self._running


class FakeRos:
    """poll_detection()이 payloads를 앞에서부터 하나씩 내주는 가짜 브리지."""

    def __init__(self, payloads=None):
        self.payloads = list(payloads or [])
        self.published = 0
        self.polls = 0

    def poll_detection(self):
        self.polls += 1
        return self.payloads.pop(0) if self.payloads else None

    def publish_conveyor_stopped(self):
        self.published += 1


def detection(size):
    return json.dumps({"box": {"id": 0, "size_m": list(size)}})


class IntakeStationTest(unittest.TestCase):
    def make(self, payloads=None, settle=3, timeout=5, empty_retries=2):
        conveyor = FakeConveyor(running=False)
        ros = FakeRos(payloads)
        station = IntakeStation(
            conveyor,
            ros,
            settle_steps=settle,
            timeout_steps=timeout,
            empty_retry_count=empty_retries,
        )
        return station, ros

    def test_publishes_trigger_after_settle(self):
        station, ros = self.make(settle=3)
        for _ in range(2):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 0)
        station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 1)
        self.assertTrue(station.trigger_sent)

    def test_zero_settle_publishes_trigger_immediately(self):
        station, ros = self.make(settle=0)
        station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 1)
        self.assertTrue(station.trigger_sent)

        # 같은 박스에 대한 요청은 한 번만 발행한다.
        station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 1)

    def test_trigger_flag_resets_for_next_box(self):
        station, _ = self.make(settle=1)
        station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertTrue(station.trigger_sent)
        station.update("/World/AutoSpawnedCubes/Cube_02", True)
        self.assertTrue(station.trigger_sent)
        station.reset()
        self.assertFalse(station.trigger_sent)

    def test_does_not_settle_while_belt_runs(self):
        station, ros = self.make(settle=3)
        station.conveyor._running = True
        for _ in range(10):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 0)
        self.assertEqual(station.state, IntakeStation.WAITING)

    def test_ready_on_valid_detection(self):
        # 1호 공칭 = 0.22 x 0.19 x 0.09
        station, _ = self.make(
            payloads=[None, None, None, detection((0.19, 0.221, 0.09))], settle=3
        )
        for _ in range(4):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(station.state, IntakeStation.READY)
        self.assertEqual(station.number, 1)
        self.assertEqual(station.dimensions, (0.22, 0.19, 0.09))
        self.assertFalse(station.timed_out)

    def test_failed_on_timeout(self):
        station, _ = self.make(payloads=None, settle=3, timeout=5)
        for _ in range(20):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(station.state, IntakeStation.FAILED)
        self.assertTrue(station.timed_out)

    def test_retries_null_box_then_becomes_ready(self):
        payload = json.dumps({"box": None})
        station, ros = self.make(
            payloads=[None, None, None, payload, detection((0.22, 0.19, 0.09))],
            settle=3,
        )
        for _ in range(5):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(station.state, IntakeStation.READY)
        self.assertEqual(station.empty_retries, 1)
        self.assertEqual(ros.published, 2)

    def test_failed_after_null_box_retry_budget_is_exhausted(self):
        payload = json.dumps({"box": None})
        station, ros = self.make(
            payloads=[None, None, None, payload, payload, payload], settle=3
        )
        for _ in range(6):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(station.state, IntakeStation.FAILED)
        self.assertFalse(station.timed_out)
        self.assertEqual(station.empty_retries, 2)
        self.assertEqual(ros.published, 3)

    def test_unknown_size_does_not_retry(self):
        station, ros = self.make(
            payloads=[None, None, None, detection((1.0, 1.0, 1.0))], settle=3
        )
        for _ in range(4):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(station.state, IntakeStation.FAILED)
        self.assertEqual(station.empty_retries, 0)
        self.assertEqual(ros.published, 1)

    def test_does_not_measure_before_reaching_pick_zone(self):
        """회귀: 벨트가 멈췄어도 Pick Zone 밖이면 재지 않는다.

        벨트는 로봇이 박스를 집는 동안에도 선다. 그때 재면 아직 스폰 지점에
        있는 다음 박스나 시야에 들어온 로봇 팔을 재고, 그 오분류가 place
        높이를 틀리게 만들어 박스를 팔레트 밖으로 떨어뜨린다.
        """
        station, ros = self.make(settle=3)
        self.assertFalse(station.conveyor.is_running())  # 벨트는 정지 상태
        for _ in range(20):
            station.update("/World/AutoSpawnedCubes/Cube_01", False)
        self.assertEqual(ros.published, 0)
        self.assertEqual(station.state, IntakeStation.WAITING)
        self.assertEqual(station.steps, 0)

        # Pick Zone에 들어오면 그때부터 센다.
        for _ in range(3):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 1)

    def test_resets_when_front_box_changes(self):
        station, ros = self.make(settle=3)
        for _ in range(3):
            station.update("/World/AutoSpawnedCubes/Cube_01", True)
        self.assertEqual(ros.published, 1)

        # 새 박스로 바뀌면 상태와 측정 결과가 지워지고 안정화가 처음부터 다시
        # 시작된다. 벨트는 이미 서 있으므로 이 호출부터 바로 세기 시작한다.
        station.update("/World/AutoSpawnedCubes/Cube_02", True)
        self.assertEqual(station.state, IntakeStation.WAITING)
        self.assertEqual(station.path, "/World/AutoSpawnedCubes/Cube_02")
        self.assertIsNone(station.number)
        self.assertIsNone(station.dimensions)

        # 앞 박스의 트리거가 재사용되지 않고, 새 박스용으로 한 번 더 나간다.
        station.update("/World/AutoSpawnedCubes/Cube_02", True)
        self.assertEqual(ros.published, 1)
        station.update("/World/AutoSpawnedCubes/Cube_02", True)
        self.assertEqual(ros.published, 2)


if __name__ == "__main__":
    unittest.main()
