import unittest
from types import SimpleNamespace

import numpy as np

from h2017_palletizing.config import BOX_CATALOG
from h2017_palletizing.planning import (
    DeepPack3DPlanner,
    PackingItem,
    PackingPlanError,
    compute_layer_slots,
    rotate_quaternion_about_world_z,
)


PALLET_AABB = (0.0, 0.0, 0.0, 1.02, 1.02, 0.15)


class DeepPack3DPlannerTests(unittest.TestCase):
    def make_planner(self, **kwargs):
        return DeepPack3DPlanner(
            PALLET_AABB,
            resolution=0.01,
            max_stack_height=1.0,
            edge_margin=0.01,
            box_gap=0.0,
            **kwargs,
        )

    def test_places_boxes_inside_pallet_without_overlap(self):
        items = [PackingItem(f"box-{index}", (0.5, 0.5, 0.2)) for index in range(8)]
        plan = self.make_planner(method="baf", lookahead=4).plan(items)

        self.assertEqual(len(plan.placements), 8)
        for placement in plan.placements:
            box = placement.grid_cuboid
            self.assertLessEqual(box.right, 100)
            self.assertLessEqual(box.front, 100)
            self.assertLessEqual(box.top, 100)
        for index, left in enumerate(plan.placements):
            for right in plan.placements[index + 1 :]:
                self.assertFalse(left.grid_cuboid.intersects(right.grid_cuboid))

    def test_lookahead_can_skip_an_item_that_is_temporarily_unplaceable(self):
        items = [
            PackingItem("too-large", (1.2, 0.2, 0.2)),
            PackingItem("fits", (0.4, 0.4, 0.2)),
        ]
        plan = self.make_planner(method="bl", lookahead=2).plan(items, require_all=False)

        self.assertEqual([p.item.item_id for p in plan.placements], ["fits"])
        self.assertEqual([item.item_id for item in plan.unplaced], ["too-large"])

    def test_requires_more_than_half_of_footprint_as_support(self):
        items = [
            PackingItem("base-0", (0.5, 0.5, 0.2)),
            PackingItem("base-1", (0.5, 0.5, 0.2)),
            PackingItem("base-2", (0.5, 0.5, 0.2)),
            PackingItem("base-3", (0.5, 0.5, 0.2)),
            PackingItem("top", (1.0, 1.0, 0.2)),
        ]
        plan = self.make_planner(method="bl", lookahead=1).plan(items)
        top = next(p for p in plan.placements if p.item.item_id == "top")
        self.assertGreater(top.support_ratio, 0.5)
        self.assertAlmostEqual(top.world_bottom_center[2], PALLET_AABB[5] + 0.2)

    def test_raises_with_partial_plan_when_capacity_is_insufficient(self):
        items = [PackingItem(f"box-{index}", (1.0, 1.0, 0.6)) for index in range(2)]
        with self.assertRaises(PackingPlanError) as caught:
            self.make_planner(method="bssf", lookahead=2).plan(items)
        self.assertEqual(len(caught.exception.partial_plan.placements), 1)
        self.assertEqual(len(caught.exception.partial_plan.unplaced), 1)

    def test_plan_is_deterministic(self):
        items = [
            PackingItem("a", (0.4, 0.6, 0.2)),
            PackingItem("b", (0.6, 0.4, 0.2)),
            PackingItem("c", (0.3, 0.3, 0.3)),
        ]
        planner = self.make_planner(method="blsf", lookahead=3)
        first = planner.plan(items)
        second = planner.plan(items)
        self.assertEqual(first.placements, second.placements)


    def test_world_z_ninety_degrees_from_identity(self):
        rotated = rotate_quaternion_about_world_z((1.0, 0.0, 0.0, 0.0), 90)
        expected = 2.0 ** -0.5
        self.assertTrue(
            np.allclose(rotated, (expected, 0.0, 0.0, expected))
        )

    def test_world_z_minus_ninety_degrees_for_robot_2(self):
        rotated = rotate_quaternion_about_world_z((1.0, 0.0, 0.0, 0.0), -90)
        expected = 2.0 ** -0.5
        self.assertTrue(
            np.allclose(rotated, (expected, 0.0, 0.0, -expected))
        )

    def test_zero_yaw_only_normalizes(self):
        quaternion = np.array((0.0, 2.0, 2.0, 0.0))
        rotated = rotate_quaternion_about_world_z(quaternion, 0)
        self.assertTrue(np.allclose(rotated, quaternion / np.linalg.norm(quaternion)))

    def test_asymmetric_y_margins_keep_only_the_outer_clearance(self):
        planner = self.make_planner(
            method="bl", lookahead=1, min_y_margin=0.0, max_y_margin=0.02
        )
        placement = planner.plan(
            [PackingItem("box", (0.2, 0.3, 0.1))]
        ).placements[0]

        self.assertAlmostEqual(placement.world_center[1] - 0.15, PALLET_AABB[1])
        self.assertEqual(planner.grid_size[1], 100)

    def test_reverse_y_starts_at_the_max_y_boundary(self):
        planner = self.make_planner(
            method="bl",
            lookahead=1,
            min_y_margin=0.02,
            max_y_margin=0.0,
            reverse_y=True,
        )
        placement = planner.plan(
            [PackingItem("box", (0.2, 0.3, 0.1))]
        ).placements[0]

        self.assertAlmostEqual(placement.world_center[1] + 0.15, PALLET_AABB[4])

def fake_placement(sequence_index, bottom_z):
    """compute_layer_slots가 읽는 두 필드만 가진 가짜 배치."""
    return SimpleNamespace(
        sequence_index=sequence_index,
        world_bottom_center=(0.0, 0.0, bottom_z),
    )


class ComputeLayerSlotsTests(unittest.TestCase):
    def test_single_layer_numbers_slots_in_sequence_order(self):
        placements = [fake_placement(index, 0.15) for index in range(3)]
        self.assertEqual(compute_layer_slots(placements), [(0, 0), (0, 1), (0, 2)])

    def test_higher_boxes_land_on_later_layers(self):
        placements = [
            fake_placement(0, 0.15),
            fake_placement(1, 0.15),
            fake_placement(2, 0.30),
        ]
        self.assertEqual(compute_layer_slots(placements), [(0, 0), (0, 1), (1, 0)])

    def test_sub_millimetre_height_differences_share_a_layer(self):
        placements = [fake_placement(0, 0.1500), fake_placement(1, 0.1502)]
        self.assertEqual(compute_layer_slots(placements), [(0, 0), (0, 1)])

    def test_slots_follow_sequence_index_not_list_order(self):
        placements = [fake_placement(2, 0.15), fake_placement(0, 0.15), fake_placement(1, 0.15)]
        self.assertEqual(compute_layer_slots(placements), [(0, 2), (0, 0), (0, 1)])

    def test_empty_input_returns_empty_list(self):
        self.assertEqual(compute_layer_slots([]), [])


# 2026-08-09 Isaac 스테이지 실측값. 팔레트 AABB를 중앙 y(-0.7988)로 가른 북쪽 반.
MEASURED_HALF_AABB = (0.6971, -0.7988, 0.0, 1.7, -0.3135, 0.1425)


class MeasuredPalletCatalogTests(unittest.TestCase):
    """실측 반쪽 팔레트에 1~4호를 적재할 수 있는지 고정한다."""

    def make_planner(self, lookahead):
        # config.py의 DEEPPACK3D_* 실제 값과 application.py의 호출 인자를 그대로 쓴다.
        return DeepPack3DPlanner(
            MEASURED_HALF_AABB,
            method="baf",
            lookahead=lookahead,
            resolution=0.01,
            max_stack_height=0.8,
            edge_margin=0.015,
            box_gap=0.015,
            min_support_ratio=0.5,
            allow_yaw_rotation=False,
        )

    def make_items(self, count):
        return [
            PackingItem(f"Cube_{index:02d}", BOX_CATALOG[(index % 4) + 1])
            for index in range(count)
        ]

    def test_six_boxes_all_fit(self):
        items = self.make_items(6)
        plan = self.make_planner(len(items)).plan(items, require_all=False)
        self.assertEqual(len(plan.placements), 6)
        self.assertEqual(len(plan.unplaced), 0)

    def test_largest_box_is_placeable(self):
        # 4호는 짧은 변에 33셀이 필요하고 가용은 45셀이다.
        items = [PackingItem("Cube_00", BOX_CATALOG[4])]
        plan = self.make_planner(1).plan(items, require_all=True)
        self.assertEqual(len(plan.placements), 1)

    def test_exact_grid_height_does_not_gain_an_extra_cell(self):
        # 0.28 / 0.01은 일부 계산에서 28.000000000000004가 된다. 두 4호를
        # 쌓았을 때 두 번째 바닥은 0.29 m가 아니라 실제 높이인 0.28 m여야 한다.
        stacking_pallet = (0.0, 0.0, 0.0, 0.46, 0.371, 0.10)
        planner = DeepPack3DPlanner(
            stacking_pallet,
            method="baf",
            lookahead=2,
            resolution=0.01,
            max_stack_height=0.8,
            edge_margin=0.015,
            box_gap=0.015,
            min_support_ratio=0.5,
            allow_yaw_rotation=False,
        )
        items = [
            PackingItem("bottom", BOX_CATALOG[4]),
            PackingItem("top", BOX_CATALOG[4]),
        ]
        plan = planner.plan(items, require_all=True)
        top = next(p for p in plan.placements if p.item.item_id == "top")
        self.assertAlmostEqual(
            top.world_bottom_center[2] - stacking_pallet[5],
            0.28,
            places=9,
        )

    def test_exact_pallet_extent_keeps_its_last_grid_cell(self):
        # 0.36 - 양쪽 margin 0.015 = 정확히 0.33 m여야 하며, 표현 오차로
        # 32칸이 되어 4호(깊이 점유 33칸)를 거부하면 안 된다.
        planner = DeepPack3DPlanner(
            (0.0, 0.0, 0.0, 0.46, 0.36, 0.10),
            resolution=0.01,
            max_stack_height=0.8,
            edge_margin=0.015,
            box_gap=0.015,
        )
        plan = planner.plan(
            [PackingItem("box", BOX_CATALOG[4])], require_all=True
        )
        self.assertEqual(len(plan.placements), 1)

    def test_placements_stay_inside_the_half_pallet(self):
        items = self.make_items(8)
        plan = self.make_planner(len(items)).plan(items, require_all=False)
        self.assertGreater(len(plan.placements), 0)
        for placement in plan.placements:
            for axis in (0, 1):
                low = placement.world_center[axis] - placement.oriented_size[axis] / 2.0
                high = placement.world_center[axis] + placement.oriented_size[axis] / 2.0
                self.assertGreaterEqual(low, MEASURED_HALF_AABB[axis] - 1e-9)
                self.assertLessEqual(high, MEASURED_HALF_AABB[axis + 3] + 1e-9)

    def test_two_halves_start_at_the_center_without_a_gap(self):
        mid_y = MEASURED_HALF_AABB[1]
        south_half = (
            MEASURED_HALF_AABB[0],
            mid_y - (MEASURED_HALF_AABB[4] - mid_y),
            MEASURED_HALF_AABB[2],
            MEASURED_HALF_AABB[3],
            mid_y,
            MEASURED_HALF_AABB[5],
        )
        common = dict(
            method="bl",
            lookahead=1,
            resolution=0.01,
            max_stack_height=0.8,
            edge_margin=0.015,
            box_gap=0.015,
        )
        north = DeepPack3DPlanner(
            MEASURED_HALF_AABB,
            min_y_margin=0.0,
            max_y_margin=0.015,
            **common,
        ).plan([PackingItem("north", BOX_CATALOG[1])]).placements[0]
        south = DeepPack3DPlanner(
            south_half,
            min_y_margin=0.015,
            max_y_margin=0.0,
            reverse_y=True,
            **common,
        ).plan([PackingItem("south", BOX_CATALOG[1])]).placements[0]

        north_inner_edge = north.world_center[1] - north.oriented_size[1] * 0.5
        south_inner_edge = south.world_center[1] + south.oriented_size[1] * 0.5
        self.assertAlmostEqual(north_inner_edge, mid_y)
        self.assertAlmostEqual(south_inner_edge, mid_y)

    def test_eight_boxes_is_the_practical_ceiling(self):
        # 팔레트 면적이 아니라 max_stack_height=0.8에 걸린다.
        # 이 상한이 바뀌면 --box-count 권장값도 다시 정해야 한다.
        eight = self.make_items(8)
        self.assertEqual(
            len(self.make_planner(8).plan(eight, require_all=False).unplaced), 0
        )

        twelve = self.make_items(12)
        self.assertGreater(
            len(self.make_planner(12).plan(twelve, require_all=False).unplaced), 0
        )


class PackingSessionTest(unittest.TestCase):
    """온라인 세션은 배치를 하나씩 확정하고 이전 결과를 보존한다."""

    def make_planner(self, **kwargs):
        return DeepPack3DPlanner(
            PALLET_AABB,
            resolution=0.01,
            max_stack_height=1.0,
            edge_margin=0.01,
            box_gap=0.0,
            **kwargs,
        )

    def items(self):
        return [
            PackingItem("Cube_01", (0.3, 0.2, 0.2)),
            PackingItem("Cube_02", (0.25, 0.25, 0.15)),
            PackingItem("Cube_03", (0.4, 0.3, 0.25)),
            PackingItem("Cube_04", (0.2, 0.2, 0.1)),
        ]

    def test_session_matches_batch_plan(self):
        """전부 배치 가능한 박스열이면 세션 결과 == lookahead=1 일괄 결과.

        일괄 plan()은 첫 실패에서 루프를 끊고, 세션은 그 박스만 버리고 계속
        간다. 그 차이가 안 드러나도록 여기서는 다 들어가는 박스만 쓴다.
        """
        batch = self.make_planner(method="bl", lookahead=1).plan(
            self.items(), require_all=False
        )
        session = self.make_planner(method="bl", lookahead=1).open_session()
        online = [session.place(item) for item in self.items()]

        self.assertNotIn(None, online)
        self.assertEqual(
            [p.item.item_id for p in online],
            [p.item.item_id for p in batch.placements],
        )
        self.assertEqual(
            [p.world_center for p in online],
            [p.world_center for p in batch.placements],
        )
        self.assertEqual(
            [p.grid_cuboid for p in online],
            [p.grid_cuboid for p in batch.placements],
        )

    def test_earlier_placements_never_move(self):
        session = self.make_planner(method="baf", lookahead=1).open_session()
        first = session.place(PackingItem("Cube_01", (0.3, 0.2, 0.2)))
        snapshot = first.world_center

        for index in range(2, 6):
            session.place(PackingItem(f"Cube_{index:02d}", (0.25, 0.2, 0.15)))

        self.assertEqual(session.placements[0].world_center, snapshot)
        self.assertEqual(session.placements[0].item.item_id, "Cube_01")

    def test_returns_none_when_pallet_is_full(self):
        session = self.make_planner(method="bl", lookahead=1).open_session()
        oversized = PackingItem("Cube_99", (5.0, 5.0, 0.2))
        self.assertIsNone(session.place(oversized))
        self.assertEqual(session.placements, [])

    def test_rejects_duplicate_item_id(self):
        session = self.make_planner(method="bl", lookahead=1).open_session()
        session.place(PackingItem("Cube_01", (0.2, 0.2, 0.1)))
        with self.assertRaises(ValueError):
            session.place(PackingItem("Cube_01", (0.2, 0.2, 0.1)))

    def test_can_place_does_not_mutate_session(self):
        session = self.make_planner(method="baf", lookahead=1).open_session()
        first = session.place(PackingItem("Cube_01", (0.3, 0.2, 0.2)))
        placements_before = list(session.placements)
        offered_before = session._offered

        self.assertTrue(
            session.can_place(PackingItem("preview", (0.25, 0.25, 0.15)))
        )
        self.assertEqual(session.placements, placements_before)
        self.assertEqual(session._offered, offered_before)
        self.assertEqual(session.placements[0], first)

    def test_can_place_reports_full_without_mutating_session(self):
        session = self.make_planner(method="bl", lookahead=1).open_session()
        self.assertFalse(
            session.can_place(PackingItem("preview", (5.0, 5.0, 0.2)))
        )
        self.assertEqual(session.placements, [])
        self.assertEqual(session._offered, 0)


if __name__ == "__main__":
    unittest.main()
