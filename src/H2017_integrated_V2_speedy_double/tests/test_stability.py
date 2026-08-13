import math
import unittest

import numpy as np

from h2017_palletizing.stability import (
    BoxPoseSample,
    BoxStabilityResult,
    assess_box_pose,
    assess_release_pose,
    box_tilt_degrees,
    is_drift_only_failure,
    pose_motion_rates,
)


class BoxStabilityTest(unittest.TestCase):
    def test_detects_lateral_only_failure(self):
        eligible = BoxStabilityResult("box", 0.006, 0.001, 1.0, True, ())
        tilted = BoxStabilityResult("box", 0.006, 0.001, 4.0, False, ("tilt",))
        kwargs = dict(
            trigger_drift_m=0.005,
            max_support_height_error_m=0.005,
            max_tilt_deg=2.0,
        )
        self.assertTrue(is_drift_only_failure(eligible, **kwargs))
        self.assertFalse(is_drift_only_failure(tilted, **kwargs))

    def assess(self, sample):
        return assess_box_pose(
            "Cube_01",
            sample,
            expected_center=(1.0, -0.5, 0.25),
            box_height_m=0.20,
            max_horizontal_drift_m=0.010,
            max_support_height_error_m=0.005,
            max_tilt_deg=2.0,
        )

    def test_upright_box_on_planned_support_passes(self):
        result = self.assess(BoxPoseSample((1.003, -0.5, 0.25), (0, 0, 1), 0.15))
        self.assertTrue(result.passed)
        self.assertEqual(result.reasons, ())

    def test_horizontal_drift_below_hard_limit_passes(self):
        result = self.assess(BoxPoseSample((1.006, -0.5, 0.25), (0, 0, 1), 0.15))
        self.assertTrue(result.passed)

    def test_horizontal_drift_above_hard_limit_is_reported(self):
        result = self.assess(BoxPoseSample((1.011, -0.5, 0.25), (0, 0, 1), 0.15))
        self.assertFalse(result.passed)
        self.assertIn("XY drift", result.reasons[0])

    def test_support_gap_and_fall_are_distinguished(self):
        high = self.assess(BoxPoseSample((1, -0.5, 0.26), (0, 0, 1), 0.16))
        low = self.assess(BoxPoseSample((1, -0.5, 0.24), (0, 0, 1), 0.14))
        self.assertIn("지지면 위 이격", high.reasons[0])
        self.assertIn("지지면 관통/낙하", low.reasons[0])

    def test_tilt_uses_box_up_axis(self):
        angle = math.radians(3.0)
        up = (math.sin(angle), 0.0, math.cos(angle))
        result = self.assess(BoxPoseSample((1, -0.5, 0.25), up, 0.15))
        self.assertAlmostEqual(box_tilt_degrees(up), 3.0, places=6)
        self.assertFalse(result.passed)
        self.assertIn("tilt", result.reasons[0])

    def test_invalid_up_axis_never_passes(self):
        result = self.assess(BoxPoseSample((1, -0.5, 0.25), (0, 0, 0), 0.15))
        self.assertFalse(result.passed)

    def test_release_rejects_penetration_and_excessive_drop(self):
        kwargs = dict(
            expected_center=(1.0, -0.5, 0.25),
            box_height_m=0.20,
            min_gap_m=0.0,
            max_gap_m=0.012,
            max_horizontal_error_m=0.015,
            max_tilt_deg=2.0,
        )
        penetrating = assess_release_pose(
            "Cube_01", BoxPoseSample((1, -0.5, 0.249), (0, 0, 1), 0.149), **kwargs
        )
        too_high = assess_release_pose(
            "Cube_01", BoxPoseSample((1, -0.5, 0.265), (0, 0, 1), 0.165), **kwargs
        )
        safe = assess_release_pose(
            "Cube_01", BoxPoseSample((1, -0.5, 0.254), (0, 0, 1), 0.154), **kwargs
        )
        self.assertIn("관통", penetrating.reasons[0])
        self.assertIn("릴리스 높이", too_high.reasons[0])
        self.assertTrue(safe.passed)

    def test_pose_motion_rates_measure_translation_and_tilt(self):
        previous = BoxPoseSample((0, 0, 0), (0, 0, 1), 0)
        angle = math.radians(1.0)
        current = BoxPoseSample(
            (0.001, 0, 0), (math.sin(angle), 0, math.cos(angle)), 0
        )
        linear, angular = pose_motion_rates(previous, current, 0.1)
        self.assertAlmostEqual(linear, 0.01)
        self.assertAlmostEqual(angular, 10.0, places=6)

    def test_pose_motion_rates_reject_invalid_dt(self):
        sample = BoxPoseSample((0, 0, 0), (0, 0, 1), 0)
        with self.assertRaises(ValueError):
            pose_motion_rates(sample, sample, 0.0)


if __name__ == "__main__":
    unittest.main()
