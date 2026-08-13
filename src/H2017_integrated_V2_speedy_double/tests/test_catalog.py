import json
import unittest

from h2017_palletizing.catalog import (
    is_empty_detection,
    parse_detection,
    snap_to_catalog,
)
from h2017_palletizing.config import BOX_CATALOG


class ParseDetectionTests(unittest.TestCase):
    def make_payload(self, size_m):
        # conveyor_box_measurement_node._publish_detections가 내는 형식 그대로.
        return json.dumps(
            {
                "stamp": 1786174308,
                "box": {
                    "id": 3,
                    "center_m": [0.2374, -0.7113, 0.7872],
                    "size_m": list(size_m),
                    "yaw_rad": -1.5708,
                },
            }
        )

    def test_valid_payload_snaps(self):
        self.assertEqual(
            parse_detection(self.make_payload(BOX_CATALOG[2])), (2, BOX_CATALOG[2])
        )

    def test_null_box_returns_none(self):
        # 비전이 아무것도 못 찾으면 box가 null로 온다.
        self.assertIsNone(parse_detection(json.dumps({"stamp": 1, "box": None})))
        self.assertTrue(is_empty_detection(json.dumps({"stamp": 1, "box": None})))

    def test_unknown_size_returns_none(self):
        payload = self.make_payload((1.0, 1.0, 1.0))
        self.assertIsNone(parse_detection(payload))
        self.assertFalse(is_empty_detection(payload))

    def test_malformed_json_raises(self):
        with self.assertRaises(ValueError):
            parse_detection("이건 JSON이 아니다")


class SnapToCatalogTests(unittest.TestCase):
    def test_exact_dimensions_snap_to_their_own_number(self):
        for number, dimensions in BOX_CATALOG.items():
            with self.subTest(number=number):
                self.assertEqual(snap_to_catalog(dimensions), (number, dimensions))

    def test_axis_order_does_not_matter(self):
        # 비전은 cv2.minAreaRect를 쓰므로 가로/세로 순서가 보장되지 않는다.
        long_side, short_side, height = BOX_CATALOG[3]
        for permutation in (
            (short_side, long_side, height),
            (height, short_side, long_side),
            (height, long_side, short_side),
        ):
            with self.subTest(permutation=permutation):
                self.assertEqual(snap_to_catalog(permutation), (3, BOX_CATALOG[3]))

    def test_small_measurement_error_still_snaps(self):
        long_side, short_side, height = BOX_CATALOG[4]
        measured = (long_side + 0.008, short_side - 0.006, height + 0.005)
        self.assertEqual(snap_to_catalog(measured), (4, BOX_CATALOG[4]))

    def test_returns_catalog_order_not_sorted_order(self):
        # 반환 치수의 [1]번 값이 벨트 폭 방향이 되므로 정렬본을 돌려주면 안 된다.
        number, dimensions = snap_to_catalog((0.09, 0.22, 0.19))
        self.assertEqual(number, 1)
        self.assertEqual(dimensions, BOX_CATALOG[1])

    def test_unknown_box_returns_none(self):
        self.assertIsNone(snap_to_catalog((1.0, 1.0, 1.0)))

    def test_midpoint_between_two_numbers_is_rejected(self):
        # 3호와 4호의 정확한 중점은 양쪽 모두 0.0579 떨어져 임계값(0.05)을 넘는다.
        self.assertIsNone(snap_to_catalog((0.375, 0.28, 0.245)))

    def test_slightly_toward_a_number_snaps_to_it(self):
        self.assertEqual(snap_to_catalog((0.40, 0.30, 0.27)), (4, BOX_CATALOG[4]))


class CatalogInvariantTests(unittest.TestCase):
    def test_long_side_comes_first(self):
        # dimensions[1]이 벨트 폭 방향이다. 긴 변이 [1]에 오면 프레임과 충돌한다.
        for number, (long_side, short_side, _height) in BOX_CATALOG.items():
            with self.subTest(number=number):
                self.assertGreaterEqual(long_side, short_side)

    def test_short_side_fits_the_belt_with_margin(self):
        # 벨트 주행면은 0.45 m. 양쪽 5 cm 이상 남아야 낙하 후 흔들려도 안전하다.
        for number, (_long_side, short_side, _height) in BOX_CATALOG.items():
            with self.subTest(number=number):
                self.assertLessEqual(short_side, 0.45 - 0.10)


if __name__ == "__main__":
    unittest.main()
