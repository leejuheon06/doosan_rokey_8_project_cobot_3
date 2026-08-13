"""BeamPackingSession: 순서를 바꾸지 않는 평가 전용 lookahead."""

import unittest

from h2017_palletizing.config import BOX_CATALOG
from h2017_palletizing.planning import (
    BeamPackingSession,
    DeepPack3DPlanner,
    PackingItem,
    StableDeepPack3DPlanner,
    lateral_support_ratio,
    support_metrics,
)


PALLET_AABB = (0.0, 0.0, 0.0, 1.12, 0.57, 0.15)


def make_planner(cls=DeepPack3DPlanner, method="baf", **kwargs):
    return cls(
        PALLET_AABB,
        method=method,
        lookahead=1,
        resolution=0.01,
        max_stack_height=0.8,
        edge_margin=0.015,
        box_gap=0.015,
        min_support_ratio=0.5,
        allow_yaw_rotation=False,
        **kwargs,
    )


def catalog_items(numbers):
    return [PackingItem(f"b{i:02d}", BOX_CATALOG[n]) for i, n in enumerate(numbers)]


class BeamSessionTests(unittest.TestCase):
    def test_open_session_returns_a_beam_session_only_when_asked(self):
        planner = make_planner()
        self.assertNotIsInstance(planner.open_session(), BeamPackingSession)
        self.assertIsInstance(planner.open_session(beam_width=3), BeamPackingSession)

    def test_rejects_a_beam_width_below_one(self):
        with self.assertRaises(ValueError):
            make_planner().open_session(beam_width=0)

    def test_without_upcoming_it_matches_the_greedy_session(self):
        items = catalog_items([1, 3, 2, 4, 2, 3, 1, 4, 3, 2, 1, 4] * 2)
        greedy = make_planner().open_session()
        beam = make_planner().open_session(beam_width=4)

        for item in items:
            greedy.place(item)
            beam.place(item)

        self.assertEqual(
            [(p.item.item_id, p.grid_cuboid) for p in greedy.placements],
            [(p.item.item_id, p.grid_cuboid) for p in beam.placements],
        )

    def test_never_reorders_even_when_a_later_box_would_score_better(self):
        # 큰 박스를 먼저 놓는 편이 점수상 유리해도 현재 박스가 나가야 한다.
        items = catalog_items([1, 4, 4, 4])
        session = make_planner().open_session(beam_width=3)

        for index, item in enumerate(items):
            placement = session.place(item, upcoming=items[index + 1 :])
            # 팔레트가 차서 못 놓을 수는 있어도, 뒤 박스가 대신 나가면 안 된다.
            if placement is not None:
                self.assertEqual(placement.item.item_id, item.item_id)

        placed_ids = [p.item.item_id for p in session.placements]
        self.assertEqual(placed_ids, sorted(placed_ids), "배치 순서가 입고 순서를 벗어났다")

    def test_lookahead_changes_where_the_current_box_goes(self):
        # 회귀 고정용. 탐색으로 찾은 실제 사례다 — 같은 입고 순서에서 beam이
        # 한 개를 더 받아낸다. 순서를 바꾼 게 아니라 놓는 자리만 달라진 결과다.
        items = catalog_items([2, 3, 4, 4, 1, 1, 4, 4, 1, 2, 4, 2, 2, 4, 2])
        greedy = make_planner(method="bl").open_session()
        beam = make_planner(method="bl").open_session(beam_width=4)

        for index, item in enumerate(items):
            greedy.place(item)
            beam.place(item, upcoming=items[index + 1 : index + 3])

        self.assertEqual(len(greedy.placements), 10)
        self.assertEqual(len(beam.placements), 11)

        beam_ids = [p.item.item_id for p in beam.placements]
        self.assertEqual(beam_ids, sorted(beam_ids), "beam이 입고 순서를 바꿨다")

    def test_beam_width_one_ignores_the_lookahead(self):
        items = catalog_items([1, 4, 4, 3, 4, 3, 4, 4])
        greedy = make_planner().open_session()
        narrow = make_planner().open_session(beam_width=1)

        for index, item in enumerate(items):
            greedy.place(item)
            narrow.place(item, upcoming=items[index + 1 :])

        self.assertEqual(
            [(p.item.item_id, p.grid_cuboid) for p in greedy.placements],
            [(p.item.item_id, p.grid_cuboid) for p in narrow.placements],
        )

    def test_rollout_does_not_leak_into_the_real_session(self):
        items = catalog_items([2, 4, 4, 3])
        session = make_planner().open_session(beam_width=4)
        session.place(items[0], upcoming=items[1:])

        self.assertEqual(len(session.placements), 1)
        self.assertEqual(len(session._partitioner.occupied), 1)
        # 롤아웃이 _offered를 건드리면 다음 박스의 input_index가 어긋난다.
        self.assertEqual(session._offered, 1)

    def test_rejects_a_duplicate_item_id(self):
        session = make_planner().open_session(beam_width=2)
        item = PackingItem("dup", BOX_CATALOG[1])
        session.place(item)
        with self.assertRaises(ValueError):
            session.place(item)

    def test_combines_with_the_stability_filter(self):
        """beam이 안정성 필터를 우회하지 못한다.

        beam은 `_score` 최솟값이 아닌 후보를 고를 수 있으므로, 필터가
        `_candidates()`에서 확실히 걸리는지 별도로 확인한다.
        """
        items = catalog_items([1, 4, 4, 3, 4, 3, 4, 4, 1, 4, 2, 3, 1, 4, 2])
        session = make_planner(
            StableDeepPack3DPlanner,
            min_horizontal_support_ratio=0.5,
            min_com_margin_ratio=0.2,
        ).open_session(beam_width=3)

        self.assertIsInstance(session, BeamPackingSession)
        checked = 0
        for index, item in enumerate(items):
            height_map = session._partitioner.height_map.copy()
            occupied = list(session._partitioner.occupied)
            placement = session.place(item, upcoming=items[index + 1 : index + 3])
            if placement is None:
                continue
            box = placement.grid_cuboid
            self.assertGreaterEqual(lateral_support_ratio(occupied, box), 0.5)
            if box.z > 0:
                _, com_margin = support_metrics(height_map, box)
                self.assertGreaterEqual(com_margin, 0.2)
                checked += 1
        self.assertTrue(checked, "상단층 배치가 없으면 검증이 무의미하다")


if __name__ == "__main__":
    unittest.main()
