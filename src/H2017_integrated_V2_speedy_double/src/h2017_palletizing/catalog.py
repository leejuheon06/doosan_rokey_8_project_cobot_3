"""택배 상자 카탈로그 스냅.

비전이 잰 치수를 알려진 규격 중 하나로 분류한다. Isaac에 의존하지 않는 순수
모듈이라 pytest로 바로 돌아간다.
"""

from __future__ import annotations

import json
import math
from typing import Iterable

from .config import BOX_CATALOG, BOX_SNAP_MAX_DISTANCE_M


def snap_to_catalog(
    measured_lwh: Iterable[float],
) -> tuple[int, tuple[float, float, float]] | None:
    """측정 치수를 가장 가까운 카탈로그 항목으로 분류한다.

    비전 노드의 size_m은 cv2.minAreaRect의 (w, l)에 높이를 붙인 것이라 축 순서가
    보장되지 않는다. 그래서 양쪽을 내림차순 정렬한 뒤 거리를 잰다.

    반환하는 치수는 정렬본이 아니라 BOX_CATALOG에 적힌 원래 순서다. 그 순서가
    벨트 폭 방향을 결정하기 때문이다.

    카탈로그에서 BOX_SNAP_MAX_DISTANCE_M보다 멀면 None을 돌려준다.
    """
    measured = sorted((float(value) for value in measured_lwh), reverse=True)
    if len(measured) != 3:
        raise ValueError(f"치수는 3개여야 합니다: {measured}")

    best_number: int | None = None
    best_distance = math.inf
    for number, dimensions in BOX_CATALOG.items():
        distance = math.dist(measured, sorted(dimensions, reverse=True))
        if distance < best_distance:
            best_distance = distance
            best_number = number

    if best_number is None or best_distance > BOX_SNAP_MAX_DISTANCE_M:
        return None
    return best_number, BOX_CATALOG[best_number]


def parse_detection(
    payload_json: str,
) -> tuple[int, tuple[float, float, float]] | None:
    """라인별 box_detections payload를 카탈로그 항목으로 바꾼다.

    박스를 못 찾았거나(box=null) 카탈로그에서 너무 멀면 None을 돌려준다.
    JSON 자체가 깨졌으면 예외를 올린다 — 그건 조용히 넘길 문제가 아니다.
    """
    payload = json.loads(payload_json)
    box = payload.get("box")
    if box is None:
        return None
    return snap_to_catalog(box["size_m"])


def is_empty_detection(payload_json: str) -> bool:
    """Return True only for a valid detection payload whose box is null."""
    return json.loads(payload_json).get("box") is None
