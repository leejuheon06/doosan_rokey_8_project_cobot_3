import unittest

import numpy as np

from h2017_palletizing.coordination import has_reached_pick_zone


class PickZoneTests(unittest.TestCase):
    def test_accepts_position_at_zone_start(self):
        self.assertTrue(has_reached_pick_zone(np.array([0.15, -0.8, 0.5])))

    def test_accepts_position_that_has_passed_zone(self):
        self.assertTrue(has_reached_pick_zone(np.array([0.50, -0.8, 0.5])))

    def test_rejects_position_before_zone(self):
        self.assertFalse(has_reached_pick_zone(np.array([0.14, -0.8, 0.5])))

    def test_rejects_position_outside_belt_y_range(self):
        self.assertFalse(has_reached_pick_zone(np.array([0.20, 0.1, 0.5])))
