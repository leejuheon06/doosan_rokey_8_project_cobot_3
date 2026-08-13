import unittest
from types import SimpleNamespace

from h2017_palletizing.coordination import (
    RegionLock,
    center_interlock_staging_box_y,
    conveyor_side_standby_target,
    footprint_intersects_center_interlock,
    quaternion_angular_error_deg,
    rank_idle_unit_indices,
)


class RegionLockTests(unittest.TestCase):
    def test_region_is_mutually_exclusive(self):
        lock = RegionLock(["pick_zone"])
        self.assertTrue(lock.acquire("pick_zone", "robot_1"))
        self.assertFalse(lock.acquire("pick_zone", "robot_2"))
        lock.release("pick_zone", "robot_1")
        self.assertTrue(lock.acquire("pick_zone", "robot_2"))

    def test_release_all_only_releases_owner_regions(self):
        lock = RegionLock(["pick_zone", "pallet"])
        lock.acquire("pick_zone", "robot_1")
        lock.acquire("pallet", "robot_2")
        lock.release_all("robot_1")
        self.assertIsNone(lock.owner["pick_zone"])
        self.assertEqual(lock.owner["pallet"], "robot_2")


class CenterInterlockTests(unittest.TestCase):
    def test_footprint_entering_center_band_requires_lock(self):
        self.assertTrue(
            footprint_intersects_center_interlock(-0.50, 0.20, -0.80, 0.25)
        )

    def test_footprint_outside_center_band_does_not_require_lock(self):
        self.assertFalse(
            footprint_intersects_center_interlock(-0.40, 0.20, -0.80, 0.25)
        )

    def test_touching_band_boundary_requires_lock(self):
        self.assertTrue(
            footprint_intersects_center_interlock(-0.45, 0.20, -0.80, 0.25)
        )

    def test_staging_payload_is_fully_outside_center_band(self):
        midpoint = 0.8
        depth = 0.31
        half_width = 0.25
        clearance = 0.08
        for base_y in (-0.4, 2.0):
            staged_y = center_interlock_staging_box_y(
                base_y, midpoint, depth, half_width, clearance
            )
            self.assertFalse(
                footprint_intersects_center_interlock(
                    staged_y, depth, midpoint, half_width
                )
            )
            nearest_edge = abs(staged_y - midpoint) - depth * 0.5
            self.assertAlmostEqual(nearest_edge - half_width, clearance)


class QuaternionAngularErrorTests(unittest.TestCase):
    def test_identity_and_ninety_degrees(self):
        self.assertAlmostEqual(
            quaternion_angular_error_deg((1, 0, 0, 0), (1, 0, 0, 0)), 0.0
        )
        self.assertAlmostEqual(
            quaternion_angular_error_deg(
                (1, 0, 0, 0), (2 ** -0.5, 0, 0, 2 ** -0.5)
            ),
            90.0,
        )

    def test_quaternion_sign_represents_the_same_pose(self):
        self.assertAlmostEqual(
            quaternion_angular_error_deg((1, 0, 0, 0), (-1, 0, 0, 0)), 0.0
        )


class IdleUnitRankingTests(unittest.TestCase):
    @staticmethod
    def unit(busy=False):
        return SimpleNamespace(busy=busy)

    @staticmethod
    def session(count):
        return SimpleNamespace(placements=[object()] * count)

    def test_prefers_the_less_loaded_pallet_half(self):
        units = [self.unit(), self.unit()]
        sessions = [self.session(3), self.session(1)]
        self.assertEqual(rank_idle_unit_indices(units, sessions), [1, 0])

    def test_excludes_busy_units(self):
        units = [self.unit(busy=True), self.unit()]
        sessions = [self.session(0), self.session(2)]
        self.assertEqual(rank_idle_unit_indices(units, sessions), [1])

    def test_tie_break_is_stable(self):
        units = [self.unit(), self.unit()]
        sessions = [self.session(2), self.session(2)]
        self.assertEqual(rank_idle_unit_indices(units, sessions), [0, 1])


class SideStandbyTargetTests(unittest.TestCase):
    def target(self, base_y):
        return conveyor_side_standby_target(
            cube_position=(0.15, -0.8, 0.85),
            robot_base_position=(0.63, base_y, 0.5),
            max_box_height=0.28,
            gripper_offset=0.12,
            approach_height=0.20,
            lateral_offset=0.55,
        )

    def test_each_robot_stages_on_its_own_side(self):
        north = self.target(0.1)
        south = self.target(-1.7)
        self.assertAlmostEqual(north[1], -0.25)
        self.assertAlmostEqual(south[1], -1.35)

    def test_uses_tallest_box_clearance(self):
        target = self.target(0.1)
        self.assertAlmostEqual(target[2], 0.85 + 0.14 + 0.12 + 0.20)
