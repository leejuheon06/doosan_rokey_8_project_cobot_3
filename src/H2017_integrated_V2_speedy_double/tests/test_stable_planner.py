"""StableDeepPack3DPlanner: 수평 지지 + 무게중심 필터를 얹은 플래너."""

import unittest

import numpy as np

from h2017_palletizing.planning import (
    DeepPack3DPlanner,
    GridCuboid,
    PackingItem,
    StableDeepPack3DPlanner,
    lateral_support_ratio,
    make_planner,
    support_metrics,
    support_edge_margin_ratio,
)


PALLET_AABB = (0.0, 0.0, 0.0, 1.02, 1.02, 0.15)


class SupportMetricsTests(unittest.TestCase):
    """바닥 지지 비율과 무게중심 여유.

    무게중심 여유는 지지영역 bbox의 짧은 변으로 정규화한다 (reference2 정의).
    중심이 지지영역 한가운데면 0.5, 경계에 걸치면 0.0, 밖이면 음수다.
    """

    def test_fully_supported_box_centres_the_com(self):
        height_map = np.full((10, 10), 5, dtype=np.int32)
        ratio, margin = support_metrics(height_map, GridCuboid(0, 0, 5, 10, 10, 3))

        self.assertAlmostEqual(ratio, 1.0)
        self.assertAlmostEqual(margin, 0.5)

    def test_half_supported_box_puts_com_on_the_support_edge(self):
        height_map = np.zeros((10, 10), dtype=np.int32)
        height_map[:, :5] = 5
        ratio, margin = support_metrics(height_map, GridCuboid(0, 0, 5, 10, 10, 3))

        self.assertAlmostEqual(ratio, 0.5)
        # 지지영역이 x<5뿐이라 중심(5.0)이 지지 경계에 정확히 걸린다.
        self.assertAlmostEqual(margin, 0.0)

    def test_unsupported_box_reports_negative_margin(self):
        height_map = np.zeros((10, 10), dtype=np.int32)
        ratio, margin = support_metrics(height_map, GridCuboid(0, 0, 5, 10, 10, 3))

        self.assertAlmostEqual(ratio, 0.0)
        self.assertLess(margin, 0.0)

    def test_off_centre_support_reduces_the_margin(self):
        height_map = np.zeros((10, 10), dtype=np.int32)
        height_map[:, 1:8] = 5
        _, margin = support_metrics(height_map, GridCuboid(0, 0, 5, 10, 10, 3))

        # 지지영역 x∈[1,8), 중심 5.0 → 여유 3, 짧은 변 7 → 3/7
        self.assertAlmostEqual(margin, 3.0 / 7.0)


class LateralSupportTests(unittest.TestCase):
    """측면 지지: -x, -y 면이 팔레트 벽/이웃 박스와 맞닿는 면적 비율.

    바닥 지지(`min_support_ratio`)와 직교하는 축이다. 교차적재가 되어 있으면
    옆에서 받쳐주므로 같은 바닥 지지율이어도 훨씬 안 넘어진다.
    """

    def test_box_in_the_corner_is_fully_braced_by_the_walls(self):
        self.assertAlmostEqual(
            lateral_support_ratio([], GridCuboid(0, 0, 0, 10, 10, 5)), 1.0
        )

    def test_box_touching_nothing_scores_zero(self):
        self.assertAlmostEqual(
            lateral_support_ratio([], GridCuboid(5, 5, 0, 10, 10, 5)), 0.0
        )

    def test_one_wall_contact_scores_half(self):
        # -x는 벽(x=0), -y는 허공.
        self.assertAlmostEqual(
            lateral_support_ratio([], GridCuboid(0, 5, 0, 10, 10, 5)), 0.5
        )

    def test_a_neighbour_braces_the_minus_x_face(self):
        neighbour = GridCuboid(0, 5, 0, 5, 10, 5)
        self.assertAlmostEqual(
            lateral_support_ratio([neighbour], GridCuboid(5, 5, 0, 10, 10, 5)), 0.5
        )

    def test_a_neighbour_only_counts_where_the_faces_overlap(self):
        # 이웃이 깊이 방향으로 절반만 겹친다 → -x 면의 절반만 인정.
        neighbour = GridCuboid(0, 5, 0, 5, 5, 5)
        self.assertAlmostEqual(
            lateral_support_ratio([neighbour], GridCuboid(5, 5, 0, 10, 10, 5)), 0.25
        )

    def test_a_neighbour_that_does_not_touch_is_ignored(self):
        detached = GridCuboid(0, 5, 0, 4, 10, 5)   # right=4, 박스 x=5 → 1칸 뜸
        self.assertAlmostEqual(
            lateral_support_ratio([detached], GridCuboid(5, 5, 0, 10, 10, 5)), 0.0
        )


class StablePlannerTests(unittest.TestCase):
    def test_support_edge_margin_distinguishes_corner_and_center(self):
        support = GridCuboid(0, 0, 0, 10, 8, 5)
        corner = GridCuboid(0, 0, 5, 6, 4, 3)
        centered = GridCuboid(2, 2, 5, 6, 4, 3)
        self.assertEqual(support_edge_margin_ratio([support], corner), 0.0)
        self.assertGreater(
            support_edge_margin_ratio([support], centered),
            support_edge_margin_ratio([support], corner),
        )

    def make(self, cls=StableDeepPack3DPlanner, **kwargs):
        return cls(
            PALLET_AABB,
            resolution=0.01,
            max_stack_height=1.0,
            edge_margin=0.01,
            box_gap=0.0,
            method="bl",
            lookahead=1,
            **kwargs,
        )

    def overhang_items(self):
        """한쪽으로만 쏠려 지지되는 2층을 만드는 최소 시나리오.

        바닥에 0.6 폭 박스를 깔면 그 위 1.0 폭 박스는 60%만 받쳐진다.
        기본 플래너의 0.5 문턱은 통과하지만 무게중심은 지지영역 경계에서
        여유 0.2밖에 없다. 남은 바닥(0.4 폭)에는 1.0 폭 박스가 못 들어가므로
        stable은 이 박스를 아예 놓지 않는다.
        """
        return [
            PackingItem("base", (0.6, 1.0, 0.2)),
            PackingItem("top", (1.0, 1.0, 0.2)),
        ]

    def test_floor_layer_is_never_rejected(self):
        # COM 여유를 최대로 요구해도 바닥층은 팔레트가 통짜로 받치므로 놓인다.
        items = [PackingItem("solo", (0.4, 0.4, 0.2))]
        plan = self.make(min_com_margin_ratio=1.0).plan(items)

        self.assertEqual(len(plan.placements), 1)

    def test_stable_placements_keep_the_com_over_the_support(self):
        items = [PackingItem(f"box-{i}", (0.33, 0.33, 0.2)) for i in range(24)]
        planner = self.make(min_horizontal_support_ratio=0.6, min_com_margin_ratio=0.2)
        plan = planner.plan(items, require_all=False)

        partitioner_heights = {}
        for placement in plan.placements:
            box = placement.grid_cuboid
            if box.z == 0:
                continue
            self.assertGreaterEqual(placement.support_ratio, 0.6)
            partitioner_heights[placement.item.item_id] = box.z
        self.assertTrue(partitioner_heights, "상단층 배치가 하나도 없으면 검증이 무의미하다")

    def test_stable_rejects_a_placement_the_default_planner_accepts(self):
        items = self.overhang_items()
        default_plan = self.make(cls=DeepPack3DPlanner).plan(items, require_all=False)
        stable_plan = self.make(
            min_horizontal_support_ratio=0.9, min_com_margin_ratio=0.3
        ).plan(items, require_all=False)

        # 기본 플래너는 60%만 받쳐진 2층을 만들고, stable은 그 후보를 버린다.
        self.assertEqual([p.item.item_id for p in default_plan.placements], ["base", "top"])
        self.assertGreater(max(p.grid_cuboid.z for p in default_plan.placements), 0)

        self.assertEqual([p.item.item_id for p in stable_plan.placements], ["base"])
        self.assertEqual([i.item_id for i in stable_plan.unplaced], ["top"])

    def test_thresholds_at_zero_reproduce_the_default_planner(self):
        items = [PackingItem(f"box-{i}", (0.24, 0.24, 0.09)) for i in range(30)]
        default_plan = self.make(cls=DeepPack3DPlanner).plan(items, require_all=False)
        stable_plan = self.make(
            min_horizontal_support_ratio=0.0, min_com_margin_ratio=-1.0
        ).plan(items, require_all=False)

        self.assertEqual(
            [(p.item.item_id, p.grid_cuboid) for p in default_plan.placements],
            [(p.item.item_id, p.grid_cuboid) for p in stable_plan.placements],
        )

    def test_rejects_out_of_range_thresholds(self):
        with self.assertRaises(ValueError):
            self.make(min_horizontal_support_ratio=1.5)
        with self.assertRaises(ValueError):
            self.make(min_com_margin_ratio=2.0)

    def test_online_session_applies_the_same_filter(self):
        session = self.make(
            min_horizontal_support_ratio=0.9, min_com_margin_ratio=0.3
        ).open_session()
        base, top = self.overhang_items()

        self.assertIsNotNone(session.place(base))
        # 일괄 plan()과 같은 필터가 걸려야 한다. can_place 미리보기도 마찬가지.
        self.assertFalse(session.can_place(top))
        self.assertIsNone(session.place(top))
        self.assertEqual([p.item.item_id for p in session.placements], ["base"])


class MakePlannerTests(unittest.TestCase):
    kwargs = dict(
        resolution=0.01, max_stack_height=1.0, edge_margin=0.01,
        box_gap=0.0, method="bl", lookahead=1,
    )

    def test_default_placer_returns_the_plain_planner(self):
        planner = make_planner(PALLET_AABB, placer="default", **self.kwargs)
        self.assertIs(type(planner), DeepPack3DPlanner)

    def test_stable_placer_returns_the_stable_planner(self):
        planner = make_planner(
            PALLET_AABB,
            placer="stable",
            min_horizontal_support_ratio=0.7,
            min_com_margin_ratio=0.1,
            **self.kwargs,
        )
        self.assertIs(type(planner), StableDeepPack3DPlanner)
        self.assertEqual(planner.min_horizontal_support_ratio, 0.7)

    def test_default_placer_ignores_the_stability_thresholds(self):
        planner = make_planner(
            PALLET_AABB,
            placer="default",
            min_horizontal_support_ratio=0.7,
            min_com_margin_ratio=0.1,
            **self.kwargs,
        )
        self.assertIs(type(planner), DeepPack3DPlanner)

    def test_rejects_an_unknown_placer(self):
        with self.assertRaises(ValueError):
            make_planner(PALLET_AABB, placer="bnb", **self.kwargs)


class CliTests(unittest.TestCase):
    """parse_args가 --placer 3종 인자를 실제로 넘겨주는지 확인한다."""

    def parse(self, argv):
        import sys
        from h2017_palletizing import config

        original = sys.argv
        sys.argv = ["test"] + argv
        try:
            return config.parse_args()
        finally:
            sys.argv = original

    def test_defaults_keep_the_original_planner(self):
        args = self.parse([])
        self.assertEqual(args.placer, "default")
        self.assertEqual(args.box_numbers, (1, 2, 3, 4))
        self.assertEqual(args.stable_min_horizontal_support, 0.3)
        self.assertEqual(args.stable_min_com_margin, 0.05)
        self.assertEqual(args.stability_policy, "strict")
        self.assertTrue(args.freeze_after_stability)

    def test_box_number_range_is_parsed(self):
        self.assertEqual(self.parse(["--box-numbers", "2-4"]).box_numbers, (2, 3, 4))

    def test_box_number_list_is_parsed_and_deduplicated(self):
        self.assertEqual(
            self.parse(["--box-numbers", "4,2,2"]).box_numbers,
            (2, 4),
        )

    def test_rejects_unknown_box_number(self):
        with self.assertRaises(SystemExit):
            self.parse(["--box-numbers", "2-5"])

    def test_stability_flags_are_parsed(self):
        args = self.parse(
            [
                "--stability-policy", "continue-drift",
                "--no-freeze-after-stability",
            ]
        )
        self.assertEqual(args.stability_policy, "continue-drift")
        self.assertFalse(args.freeze_after_stability)

    def test_removed_experimental_options_are_rejected(self):
        for option in (
            "--arm-obstacles",
            "--reconcile-settled-pose",
            "--stable-score-first",
        ):
            with self.subTest(option=option), self.assertRaises(SystemExit):
                self.parse([option])

    def test_stable_tiebreak_flag_is_parsed(self):
        self.assertTrue(self.parse(["--stable-tiebreak"]).stable_tiebreak)

    def test_flags_are_parsed(self):
        args = self.parse(
            [
                "--placer", "stable",
                "--stable-min-horizontal-support", "0.7",
                "--stable-min-com-margin", "0.2",
            ]
        )
        self.assertEqual(args.placer, "stable")
        self.assertEqual(args.stable_min_horizontal_support, 0.7)
        self.assertEqual(args.stable_min_com_margin, 0.2)

    def test_rejects_an_out_of_range_threshold(self):
        with self.assertRaises(SystemExit):
            self.parse(["--stable-min-horizontal-support", "1.5"])


if __name__ == "__main__":
    unittest.main()


class YawRotationTests(unittest.TestCase):
    """--yaw-rotation: 박스를 Z축으로 90도 돌려 놓는 선택지.

    반쪽 팔레트가 가늘고 길어 회전 없이는 큰 박스가 깊이 방향에 안 들어간다.
    """

    def make(self, yaw):
        return DeepPack3DPlanner(
            (0.0, 0.0, 0.0, 1.02, 0.52, 0.15),
            method="bssf",
            lookahead=1,
            resolution=0.01,
            max_stack_height=0.8,
            edge_margin=0.015,
            box_gap=0.015,
            min_support_ratio=0.5,
            allow_yaw_rotation=yaw,
        )

    def deep_items(self):
        # 깊이 0.485 m 영역에 0.41 x 0.31 박스를 세로로 넣으면 안 들어가지만
        # 90도 돌리면 0.31 x 0.41이 되어 들어간다.
        return [PackingItem(f"b{i}", (0.41, 0.31, 0.28)) for i in range(6)]

    def test_rotation_is_never_offered_when_disabled(self):
        session = self.make(False).open_session()
        for item in self.deep_items():
            placement = session.place(item)
            if placement is not None:
                self.assertEqual(placement.yaw_degrees, 0)

    def test_rotation_lets_more_boxes_fit(self):
        counts = []
        for yaw in (False, True):
            session = self.make(yaw).open_session()
            for item in self.deep_items():
                session.place(item)
            counts.append(len(session.placements))
        self.assertGreater(counts[1], counts[0])

    def test_square_footprints_are_never_rotated(self):
        # 정사각 발자국은 돌려도 같으므로 후보를 두 배로 늘리지 않는다.
        session = self.make(True).open_session()
        placement = session.place(PackingItem("square", (0.3, 0.3, 0.2)))
        self.assertEqual(placement.yaw_degrees, 0)

    def test_rotated_placement_reports_the_swapped_size(self):
        session = self.make(True).open_session()
        rotated = None
        for item in self.deep_items():
            placement = session.place(item)
            if placement is not None and placement.yaw_degrees == 90:
                rotated = placement
                break
        self.assertIsNotNone(rotated, "회전 배치가 하나도 안 나오면 검증이 무의미하다")
        self.assertEqual(rotated.oriented_size, (0.31, 0.41, 0.28))


class YawCliTests(unittest.TestCase):
    def parse(self, argv):
        import sys
        from h2017_palletizing import config

        original = sys.argv
        sys.argv = ["test"] + argv
        try:
            return config.parse_args()
        finally:
            sys.argv = original

    def test_rotation_is_off_by_default(self):
        # 로봇 손목 yaw 제어가 붙기 전에는 기본이 꺼져 있어야 한다.
        self.assertFalse(self.parse([]).yaw_rotation)

    def test_flag_turns_rotation_on(self):
        self.assertTrue(self.parse(["--yaw-rotation"]).yaw_rotation)
