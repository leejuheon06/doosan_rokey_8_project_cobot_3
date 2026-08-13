import math
import unittest

from h2017_palletizing.efficiency import calculate_space_efficiency


class SpaceEfficiencyTests(unittest.TestCase):
    def test_dense_half_footprint_uses_harmonic_mean(self):
        result = calculate_space_efficiency(
            [((0.25, 0.5, 0.5), (0.5, 1.0, 1.0))],
            (0.0, 0.0, 0.0, 1.0, 1.0, 0.0),
            stability_pass_count=1,
            stability_check_count=1,
        )
        self.assertAlmostEqual(result.footprint_coverage, 0.5)
        self.assertAlmostEqual(result.vertical_compactness, 1.0)
        self.assertAlmostEqual(result.project_efficiency, 2.0 / 3.0)

    def test_vertical_gap_reduces_compactness(self):
        result = calculate_space_efficiency(
            [
                ((0.5, 0.5, 0.25), (1.0, 1.0, 0.5)),
                ((0.5, 0.5, 1.25), (1.0, 1.0, 0.5)),
            ],
            (0.0, 0.0, 0.0, 1.0, 1.0, 0.0),
            stability_pass_count=2,
            stability_check_count=2,
        )
        self.assertAlmostEqual(result.footprint_coverage, 1.0)
        self.assertAlmostEqual(result.vertical_compactness, 2.0 / 3.0)
        self.assertAlmostEqual(result.occupied_surface_volume_m3, 1.5)

    def test_stability_pass_rate_scales_project_score(self):
        result = calculate_space_efficiency(
            [((0.5, 0.5, 0.5), (1.0, 1.0, 1.0))],
            (0.0, 0.0, 0.0, 1.0, 1.0, 0.0),
            stability_pass_count=3,
            stability_check_count=4,
        )
        self.assertTrue(math.isclose(result.project_efficiency, 0.75))


if __name__ == "__main__":
    unittest.main()
