"""DeepPack3D-compatible online 3-D pallet packing planner.

This module implements the constructive-heuristic path described by the
DeepPack3D project without importing TensorFlow.  It keeps the parts needed by
the H2017 simulator: maximal free-space partitioning, a height map, physical
support filtering, item lookahead and the BL/BAF/BSSF/BLSF selectors.

DeepPack3D reference implementation:
https://github.com/SoftwareImpacts/SIMPAC-2024-311
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np


SUPPORTED_METHODS = ("bl", "baf", "bssf", "blsf")

# 배치 선택기. "default"는 원본 DeepPack3D 그대로(수직 지지만 검사),
# "stable"은 거기에 수평 지지 + 무게중심 필터를 얹는다. 휴리스틱 4종
# (SUPPORTED_METHODS)과는 직교하는 축이라 어느 조합이든 쓸 수 있다.
SUPPORTED_PLACERS = ("default", "stable")


def rotate_quaternion_about_world_z(
    quaternion: Sequence[float], yaw_degrees: float
) -> np.ndarray:
    """Pre-multiply a scalar-first quaternion by a world-Z yaw rotation."""
    q = np.asarray(quaternion, dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError(f"quaternion must be a finite 4-vector: {quaternion}")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12 or not math.isfinite(yaw_degrees):
        raise ValueError("quaternion norm and yaw_degrees must be valid")
    w, x, y, z = q / norm
    half = math.radians(float(yaw_degrees)) * 0.5
    cw, sz = math.cos(half), math.sin(half)
    rotated = np.array(
        [
            cw * w - sz * z,
            cw * x - sz * y,
            cw * y + sz * x,
            cw * z + sz * w,
        ],
        dtype=np.float64,
    )
    return rotated / np.linalg.norm(rotated)


class PackingPlanError(RuntimeError):
    """Raised when one or more items cannot be placed on the pallet."""

    def __init__(self, message: str, partial_plan: "PackingPlan") -> None:
        super().__init__(message)
        self.partial_plan = partial_plan


@dataclass(frozen=True)
class PackingItem:
    """One physical box, expressed in world X/Y/Z dimensions (metres)."""

    item_id: str
    size: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.item_id:
            raise ValueError("item_id must not be empty")
        if len(self.size) != 3 or any(not math.isfinite(v) or v <= 0 for v in self.size):
            raise ValueError(f"invalid item size for {self.item_id}: {self.size}")


@dataclass(frozen=True)
class GridCuboid:
    """Axis-aligned cuboid in integer grid coordinates; Z is vertical."""

    x: int
    y: int
    z: int
    width: int
    depth: int
    height: int

    @property
    def right(self) -> int:
        return self.x + self.width

    @property
    def front(self) -> int:
        return self.y + self.depth

    @property
    def top(self) -> int:
        return self.z + self.height

    @property
    def volume(self) -> int:
        return self.width * self.depth * self.height

    def contains(self, other: "GridCuboid") -> bool:
        return (
            self.x <= other.x
            and self.y <= other.y
            and self.z <= other.z
            and self.right >= other.right
            and self.front >= other.front
            and self.top >= other.top
        )

    def intersects(self, other: "GridCuboid") -> bool:
        return (
            self.x < other.right
            and self.right > other.x
            and self.y < other.front
            and self.front > other.y
            and self.z < other.top
            and self.top > other.z
        )

    def fits_size(self, size: tuple[int, int, int]) -> bool:
        width, depth, height = size
        return self.width >= width and self.depth >= depth and self.height >= height

    def split(self, occupied: "GridCuboid") -> list["GridCuboid"]:
        """Return maximal axis-aligned free cuboids after subtracting occupied."""
        if not self.intersects(occupied):
            return [self]

        parts: list[GridCuboid] = []
        if self.x < occupied.x:
            parts.append(
                GridCuboid(self.x, self.y, self.z, occupied.x - self.x, self.depth, self.height)
            )
        if self.right > occupied.right:
            parts.append(
                GridCuboid(
                    occupied.right,
                    self.y,
                    self.z,
                    self.right - occupied.right,
                    self.depth,
                    self.height,
                )
            )
        if self.y < occupied.y:
            parts.append(
                GridCuboid(self.x, self.y, self.z, self.width, occupied.y - self.y, self.height)
            )
        if self.front > occupied.front:
            parts.append(
                GridCuboid(
                    self.x,
                    occupied.front,
                    self.z,
                    self.width,
                    self.front - occupied.front,
                    self.height,
                )
            )
        if self.z < occupied.z:
            parts.append(
                GridCuboid(self.x, self.y, self.z, self.width, self.depth, occupied.z - self.z)
            )
        if self.top > occupied.top:
            parts.append(
                GridCuboid(
                    self.x,
                    self.y,
                    occupied.top,
                    self.width,
                    self.depth,
                    self.top - occupied.top,
                )
            )
        return [part for part in parts if part.volume > 0]


@dataclass(frozen=True)
class PackingPlacement:
    item: PackingItem
    input_index: int
    sequence_index: int
    grid_cuboid: GridCuboid
    world_bottom_center: tuple[float, float, float]
    world_center: tuple[float, float, float]
    oriented_size: tuple[float, float, float]
    yaw_degrees: int
    support_ratio: float
    support_item_ids: tuple[str, ...]


@dataclass(frozen=True)
class PackingPlan:
    placements: tuple[PackingPlacement, ...]
    unplaced: tuple[PackingItem, ...]
    utilization: float
    packed_volume: float
    pallet_capacity_volume: float
    highest_top: float


@dataclass(frozen=True)
class _Candidate:
    pending_index: int
    input_index: int
    item: PackingItem
    cuboid: GridCuboid
    free_space: GridCuboid
    oriented_size: tuple[float, float, float]
    yaw_degrees: int
    support_ratio: float


class _SpacePartitioner:
    def __init__(self, size: tuple[int, int, int]) -> None:
        self.size = size
        width, depth, height = size
        self.container = GridCuboid(0, 0, 0, width, depth, height)
        self.free_spaces = [self.container]
        self.occupied: list[GridCuboid] = []
        self.height_map = np.zeros((depth, width), dtype=np.int32)

    def fits(self, cuboid: GridCuboid) -> bool:
        if not self.container.contains(cuboid):
            return False
        return not any(placed.intersects(cuboid) for placed in self.occupied)

    def add(self, cuboid: GridCuboid) -> None:
        if not self.fits(cuboid):
            raise ValueError(f"invalid DeepPack3D placement: {cuboid}")

        self.occupied.append(cuboid)
        footprint = self.height_map[cuboid.y : cuboid.front, cuboid.x : cuboid.right]
        np.maximum(footprint, cuboid.top, out=footprint)

        split_spaces: list[GridCuboid] = []
        for free_space in self.free_spaces:
            split_spaces.extend(free_space.split(cuboid))

        # Maximal-space representation: discard duplicates and every cuboid
        # fully contained by another free cuboid.
        unique_spaces = list(dict.fromkeys(split_spaces))
        maximal_spaces: list[GridCuboid] = []
        for index, space in enumerate(unique_spaces):
            if any(
                index != other_index and other.contains(space)
                for other_index, other in enumerate(unique_spaces)
            ):
                continue
            maximal_spaces.append(space)
        self.free_spaces = maximal_spaces


def support_metrics(height_map: np.ndarray, cuboid: GridCuboid) -> tuple[float, float]:
    """박스 바닥면의 (수평 지지 비율, 무게중심 여유 비율)을 잰다.

    지지영역은 박스 발자국 안에서 윗면 높이가 정확히 ``cuboid.z``인 셀들이다.

    - 수평 지지 비율: 지지 셀 수 / 발자국 셀 수.
    - 무게중심 여유 비율: 박스 중심에서 지지영역 bounding box 경계까지의 최소
      거리를 **지지영역의 짧은 변**으로 나눈 값. 0.5면 중심이 지지영역
      한가운데, 0.0이면 경계에 정확히 걸린 상태, 음수면 중심이 지지영역 밖이라
      그대로 두면 넘어진다.

    정규화 분모가 박스가 아니라 지지영역인 것은 reference2 구현(정완지표를
    만든 그 코드)과 임계값을 호환시키기 위해서다. 그쪽 실측 튜닝 결과를
    그대로 가져다 쓸 수 있어야 한다.

    지지 셀이 하나도 없으면 ``(0.0, -1.0)``을 돌려준다. 셀 경계를 반 칸씩
    쓰는 이산 좌표 대신 셀이 덮는 연속 구간 ``[i, i+1)``로 계산해야 "중심이
    경계에 걸림"이 정확히 0.0이 된다.
    """
    footprint = height_map[cuboid.y : cuboid.front, cuboid.x : cuboid.right]
    contact = footprint == cuboid.z
    ratio = float(contact.sum() / contact.size)

    rows, cols = np.nonzero(contact)
    if rows.size == 0:
        return 0.0, -1.0

    row_low, row_high = float(rows.min()), float(rows.max() + 1)
    col_low, col_high = float(cols.min()), float(cols.max() + 1)
    centre_row, centre_col = cuboid.depth / 2.0, cuboid.width / 2.0

    margin = min(
        centre_row - row_low,
        row_high - centre_row,
        centre_col - col_low,
        col_high - centre_col,
    )
    support_extent = max(min(row_high - row_low, col_high - col_low), 1e-6)
    return ratio, float(margin / support_extent)


def lateral_support_ratio(
    occupied: Sequence[GridCuboid], cuboid: GridCuboid
) -> float:
    """박스의 -x, -y 옆면이 팔레트 벽이나 이웃 박스와 맞닿는 면적 비율.

    바닥 지지(`min_support_ratio`, `support_metrics`의 첫 값)와 **직교하는
    축**이다. 옆에서 받쳐주면 같은 바닥 지지율이어도 훨씬 안 넘어진다 —
    교차적재(interlocking)가 안정성을 높이는 것이 이 원리다.

    Švaco 외, "Solving Pallet Loading Problem with Real-World Constraints"
    (arXiv:2307.11531)의 수평 지지 정의다. +x, +y 면은 논문처럼 랩이나 외곽
    지지가 있다고 보고 검사하지 않는다 — 뒤에 올 박스가 받쳐줄 수도 있어서
    지금 시점에 판정할 수 없기도 하다.
    """
    supported = 0.0
    total = 0.0

    for face_area, at_wall, matches in (
        (
            cuboid.depth * cuboid.height,
            cuboid.x == 0,
            lambda o: (
                o.right == cuboid.x,
                min(o.front, cuboid.front) - max(o.y, cuboid.y),
            ),
        ),
        (
            cuboid.width * cuboid.height,
            cuboid.y == 0,
            lambda o: (
                o.front == cuboid.y,
                min(o.right, cuboid.right) - max(o.x, cuboid.x),
            ),
        ),
    ):
        total += face_area
        if at_wall:
            supported += face_area
            continue
        for other in occupied:
            touching, span = matches(other)
            if not touching:
                continue
            height_overlap = min(other.top, cuboid.top) - max(other.z, cuboid.z)
            supported += max(0, span) * max(0, height_overlap)

    return supported / total if total else 1.0


def support_edge_margin_ratio(
    occupied: Sequence[GridCuboid], cuboid: GridCuboid
) -> float:
    """Return box-edge clearance inside the envelope of its top supports."""
    if cuboid.z == 0:
        return 0.5
    supports = [
        other for other in occupied
        if other.top == cuboid.z
        and other.x < cuboid.right and other.right > cuboid.x
        and other.y < cuboid.front and other.front > cuboid.y
    ]
    if not supports:
        return -1.0
    low_x = min(s.x for s in supports)
    low_y = min(s.y for s in supports)
    high_x = max(s.right for s in supports)
    high_y = max(s.front for s in supports)
    clearance = min(
        cuboid.x - low_x,
        high_x - cuboid.right,
        cuboid.y - low_y,
        high_y - cuboid.front,
    )
    return float(clearance / max(min(cuboid.width, cuboid.depth), 1))


class DeepPack3DPlanner:
    """Plan online pallet placements using DeepPack3D constructive heuristics.

    The pallet AABB is ``(min_x, min_y, min_z, max_x, max_y, max_z)``.
    Only yaw rotation is optionally offered because the H2017 top-down vacuum
    pick cannot turn a box onto another face without a dedicated regrasp step.
    """

    def __init__(
        self,
        pallet_aabb: Sequence[float],
        *,
        method: str = "baf",
        lookahead: int = 5,
        resolution: float = 0.01,
        max_stack_height: float = 1.0,
        edge_margin: float = 0.01,
        min_y_margin: float | None = None,
        max_y_margin: float | None = None,
        reverse_y: bool = False,
        box_gap: float = 0.01,
        min_support_ratio: float = 0.5,
        allow_yaw_rotation: bool = False,
        center_stack_candidates: bool = False,
    ) -> None:
        aabb = np.asarray(pallet_aabb, dtype=float)
        if aabb.shape != (6,) or not np.all(np.isfinite(aabb)):
            raise ValueError(f"pallet_aabb must contain six finite values: {pallet_aabb}")
        if np.any(aabb[3:] <= aabb[:3]):
            raise ValueError(f"invalid pallet AABB: {pallet_aabb}")
        if method not in SUPPORTED_METHODS:
            raise ValueError(f"method must be one of {SUPPORTED_METHODS}: {method}")
        if lookahead < 1:
            raise ValueError("lookahead must be at least 1")
        if resolution <= 0 or max_stack_height <= 0:
            raise ValueError("resolution and max_stack_height must be positive")
        min_y_margin = edge_margin if min_y_margin is None else min_y_margin
        max_y_margin = edge_margin if max_y_margin is None else max_y_margin
        if min(edge_margin, min_y_margin, max_y_margin, box_gap) < 0:
            raise ValueError("edge margins and box_gap cannot be negative")
        if not 0.0 <= min_support_ratio <= 1.0:
            raise ValueError("min_support_ratio must be between 0 and 1")

        usable_width = float(aabb[3] - aabb[0] - 2.0 * edge_margin)
        usable_depth = float(aabb[4] - aabb[1] - min_y_margin - max_y_margin)
        if usable_width <= 0 or usable_depth <= 0:
            raise ValueError("edge_margin leaves no usable pallet area")

        self.pallet_aabb = aabb
        self.method = method
        self.lookahead = lookahead
        self.resolution = resolution
        self.max_stack_height = max_stack_height
        self.edge_margin = edge_margin
        self.min_y_margin = float(min_y_margin)
        self.max_y_margin = float(max_y_margin)
        self.reverse_y = bool(reverse_y)
        self.box_gap = box_gap
        self.min_support_ratio = min_support_ratio
        self.allow_yaw_rotation = allow_yaw_rotation
        self.center_stack_candidates = center_stack_candidates
        def available_cells(length: float) -> int:
            # 정확한 격자 배수가 32.99999999999999처럼 표현되어 사용 가능한
            # 마지막 한 칸을 잃지 않도록 수치 오차만 보정한다.
            return int(math.floor(length / resolution + 1e-9))

        self.grid_size = (
            available_cells(usable_width),
            available_cells(usable_depth),
            available_cells(max_stack_height),
        )
        if min(self.grid_size) < 1:
            raise ValueError(
                f"resolution {resolution} is too coarse for pallet/grid size {self.grid_size}"
            )

    def _grid_item_size(self, size: tuple[float, float, float]) -> tuple[int, int, int]:
        def cells(length: float) -> int:
            # 0.28 / 0.01 같은 정확한 격자 배수가 이진 부동소수점에서는
            # 28.000000000000004가 될 수 있다. 그대로 ceil하면 한 칸(10 mm)을
            # 과대 점유해 2단 박스가 실제 지지면보다 떠서 배치된다.
            ratio = length / self.resolution
            return max(1, int(math.ceil(ratio - 1e-9)))

        return (
            cells(size[0] + self.box_gap),
            cells(size[1] + self.box_gap),
            cells(size[2]),
        )

    def _orientations(
        self, item: PackingItem
    ) -> Iterable[tuple[tuple[int, int, int], tuple[float, float, float], int]]:
        sx, sy, sz = item.size
        original = self._grid_item_size(item.size)
        yield original, item.size, 0
        if self.allow_yaw_rotation and not math.isclose(sx, sy):
            rotated_size = (sy, sx, sz)
            yield self._grid_item_size(rotated_size), rotated_size, 90

    def _generate_candidates(
        self,
        partitioner: _SpacePartitioner,
        pending: list[tuple[int, PackingItem]],
    ) -> list[_Candidate]:
        result: list[_Candidate] = []
        max_grid_height = partitioner.size[2]

        for pending_index, (input_index, item) in enumerate(pending[: self.lookahead]):
            for grid_size, oriented_size, yaw_degrees in self._orientations(item):
                width, depth, height = grid_size
                seen_origins: set[tuple[int, int]] = set()
                for free_space in partitioner.free_spaces:
                    # DeepPack3D only uses free partitions that remain open to
                    # the top, matching top-down robotic placement.
                    if free_space.top != max_grid_height or not free_space.fits_size(grid_size):
                        continue
                    origins = [(free_space.x, free_space.y)]
                    if self.center_stack_candidates:
                        origins.extend(
                            (
                                support.x + (support.width - width) // 2,
                                support.y + (support.depth - depth) // 2,
                            )
                            for support in partitioner.occupied
                            if support.width >= width and support.depth >= depth
                        )
                    for x, y in origins:
                        origin = (x, y)
                        if origin in seen_origins:
                            continue

                        if (
                            x < 0 or y < 0
                            or x + width > partitioner.size[0]
                            or y + depth > partitioner.size[1]
                        ):
                            continue
                        footprint = partitioner.height_map[
                            y : y + depth, x : x + width
                        ]
                        if footprint.shape != (depth, width):
                            continue
                        z = int(np.max(footprint))
                        if z + height > max_grid_height:
                            continue
                        support_ratio = float(
                            np.count_nonzero(footprint == z) / footprint.size
                        )
                        seen_origins.add(origin)
                        # The official implementation accepts support strictly
                        # greater than 50%; preserve that boundary at 0.5.
                        if support_ratio <= self.min_support_ratio:
                            continue

                        cuboid = GridCuboid(x, y, z, width, depth, height)
                        if not partitioner.fits(cuboid):
                            continue

                        containing_spaces = [
                            space for space in partitioner.free_spaces
                            if space.contains(cuboid)
                        ]
                        scoring_space = min(
                            containing_spaces or [free_space],
                            key=lambda space: space.volume,
                        )
                        result.append(
                            _Candidate(
                                pending_index=pending_index,
                                input_index=input_index,
                                item=item,
                                cuboid=cuboid,
                                free_space=scoring_space,
                                oriented_size=oriented_size,
                                yaw_degrees=yaw_degrees,
                                support_ratio=support_ratio,
                            )
                        )
        return result

    def _candidates(
        self,
        partitioner: _SpacePartitioner,
        pending: list[tuple[int, PackingItem]],
    ) -> list[_Candidate]:
        return self._generate_candidates(partitioner, pending)

    def _heuristic_score(self, candidate: _Candidate) -> tuple[float, ...]:
        box = candidate.cuboid
        free = candidate.free_space
        width_left = free.width - box.width
        depth_left = free.depth - box.depth
        tie_breaker = (
            candidate.pending_index,
            candidate.input_index,
            candidate.yaw_degrees,
            box.x,
            box.y,
        )
        if self.method == "bl":
            return (box.top, box.right, box.front, *tie_breaker)
        if self.method == "baf":
            return (free.volume, min(width_left, depth_left), *tie_breaker)
        if self.method == "bssf":
            return (min(width_left, depth_left), *tie_breaker)
        return (max(width_left, depth_left), *tie_breaker)

    def _score(self, candidate: _Candidate) -> tuple[float, ...]:
        return self._heuristic_score(candidate)

    def _place_one(
        self,
        partitioner: _SpacePartitioner,
        pending: list[tuple[int, PackingItem]],
        placements: list[PackingPlacement],
    ) -> PackingPlacement | None:
        """후보 중 최선 하나를 배치하고 pending에서 뺀다. 둘 곳이 없으면 None.

        일괄 plan()과 온라인 PackingSession이 이 한 몸을 공유한다. 배치 규칙이
        두 경로에서 갈라지지 않게 하려는 것이다.
        """
        candidates = self._candidates(partitioner, pending)
        if not candidates:
            return None
        selected = min(candidates, key=self._score)
        pending.pop(selected.pending_index)
        return self._apply_candidate(partitioner, selected, placements)

    def _apply_candidate(
        self,
        partitioner: _SpacePartitioner,
        selected: _Candidate,
        placements: list[PackingPlacement],
    ) -> PackingPlacement:
        """고른 후보를 실제로 격자에 반영하고 배치 기록을 만든다.

        후보 "선택"과 "반영"을 나눠 둔 것은 beam 세션 때문이다. 그쪽은
        `_score` 최솟값이 아닌 다른 후보를 고를 수 있어야 한다.
        """
        support_item_ids = tuple(
            placement.item.item_id
            for placement in placements
            if placement.grid_cuboid.top == selected.cuboid.z
            and placement.grid_cuboid.x < selected.cuboid.right
            and placement.grid_cuboid.right > selected.cuboid.x
            and placement.grid_cuboid.y < selected.cuboid.front
            and placement.grid_cuboid.front > selected.cuboid.y
        )
        partitioner.add(selected.cuboid)

        sx, sy, sz = selected.oriented_size
        world_y = (
            float(self.pallet_aabb[4] - self.max_y_margin)
            - selected.cuboid.y * self.resolution
            - sy / 2.0
            if self.reverse_y
            else float(self.pallet_aabb[1] + self.min_y_margin)
            + selected.cuboid.y * self.resolution
            + sy / 2.0
        )
        bottom_center = (
            float(self.pallet_aabb[0] + self.edge_margin)
            + selected.cuboid.x * self.resolution
            + sx / 2.0,
            world_y,
            float(self.pallet_aabb[5]) + selected.cuboid.z * self.resolution,
        )
        center = (bottom_center[0], bottom_center[1], bottom_center[2] + sz / 2.0)
        placement = PackingPlacement(
            item=selected.item,
            input_index=selected.input_index,
            sequence_index=len(placements),
            grid_cuboid=selected.cuboid,
            world_bottom_center=bottom_center,
            world_center=center,
            oriented_size=selected.oriented_size,
            yaw_degrees=selected.yaw_degrees,
            support_ratio=selected.support_ratio,
            support_item_ids=support_item_ids,
        )
        placements.append(placement)
        return placement

    def plan(self, items: Sequence[PackingItem], *, require_all: bool = True) -> PackingPlan:
        if len({item.item_id for item in items}) != len(items):
            raise ValueError("PackingItem.item_id values must be unique")

        partitioner = _SpacePartitioner(self.grid_size)
        pending = list(enumerate(items))
        placements: list[PackingPlacement] = []

        while pending:
            if self._place_one(partitioner, pending, placements) is None:
                break

        packed_volume = float(sum(np.prod(placement.item.size) for placement in placements))
        capacity = float(
            (self.pallet_aabb[3] - self.pallet_aabb[0] - 2.0 * self.edge_margin)
            * (
                self.pallet_aabb[4]
                - self.pallet_aabb[1]
                - self.min_y_margin
                - self.max_y_margin
            )
            * self.max_stack_height
        )
        highest_top = max(
            (placement.world_center[2] + placement.oriented_size[2] / 2.0 for placement in placements),
            default=float(self.pallet_aabb[5]),
        )
        plan = PackingPlan(
            placements=tuple(placements),
            unplaced=tuple(item for _, item in pending),
            utilization=packed_volume / capacity if capacity else 0.0,
            packed_volume=packed_volume,
            pallet_capacity_volume=capacity,
            highest_top=highest_top,
        )
        if require_all and plan.unplaced:
            ids = ", ".join(item.item_id for item in plan.unplaced)
            raise PackingPlanError(
                f"DeepPack3D could not place {len(plan.unplaced)} item(s): {ids}", plan
            )
        return plan

    def open_session(self, *, beam_width: int | None = None) -> "PackingSession":
        """온라인 입고용 세션을 연다. 로봇 1대가 하나씩 들고 쓴다.

        ``beam_width``를 주면 다음 박스들의 치수를 배치 위치 평가에만 쓰는
        beam 세션이 열린다. 안정성 필터와는 직교하므로 어느 플래너에서든
        쓸 수 있다.
        """
        if beam_width is None:
            return PackingSession(self)
        return BeamPackingSession(self, beam_width=beam_width)


class PackingSession:
    """박스를 하나씩 받아 배치하고 그 상태를 유지하는 온라인 패커.

    이미 놓인 박스는 다시 계산하지 않는다. 실물이 팔레트에 올라가 있으므로
    좌표가 바뀌면 안 된다. pending에 항상 한 개만 넣으므로 planner의 lookahead
    값과 무관하게 K=1로 동작한다.
    """

    def __init__(self, planner: DeepPack3DPlanner) -> None:
        self.planner = planner
        self.placements: list[PackingPlacement] = []
        self._partitioner = _SpacePartitioner(planner.grid_size)
        self._offered = 0

    def place(self, item: PackingItem) -> PackingPlacement | None:
        """박스 하나를 배치한다. 둘 곳이 없으면 None (상태는 안 바뀐다)."""
        if any(p.item.item_id == item.item_id for p in self.placements):
            raise ValueError(f"이미 배치된 item_id입니다: {item.item_id}")
        pending = [(self._offered, item)]
        self._offered += 1
        return self.planner._place_one(self._partitioner, pending, self.placements)

    def can_place(self, item: PackingItem) -> bool:
        """현재 상태를 바꾸지 않고 다음 박스의 배치 가능 여부를 확인한다.

        idle/busy 반쪽의 수용 가능성을 미리 검사할 때 사용한다. 실제 세션의
        free-space, height map, placements와 sequence counter를 건드리면 preview
        자체가 다음 실제 배치를 바꾸므로 필요한 상태만 복제해 `_place_one`을 실행한다.
        """
        if any(p.item.item_id == item.item_id for p in self.placements):
            raise ValueError(f"이미 배치된 item_id입니다: {item.item_id}")

        preview_partitioner = _SpacePartitioner(self.planner.grid_size)
        preview_partitioner.free_spaces = list(self._partitioner.free_spaces)
        preview_partitioner.occupied = list(self._partitioner.occupied)
        preview_partitioner.height_map = self._partitioner.height_map.copy()
        preview_placements = list(self.placements)
        pending = [(self._offered, item)]
        return (
            self.planner._place_one(
                preview_partitioner, pending, preview_placements
            )
            is not None
        )

def _clone_partitioner(
    source: _SpacePartitioner, grid_size: tuple[int, int, int]
) -> _SpacePartitioner:
    """롤아웃용 격자 사본. free space 목록의 원소는 frozen이라 얕은 복사로 족하다."""
    clone = _SpacePartitioner(grid_size)
    clone.free_spaces = list(source.free_spaces)
    clone.occupied = list(source.occupied)
    clone.height_map = source.height_map.copy()
    return clone


class BeamPackingSession(PackingSession):
    """다음 박스들을 미리 보되, 놓는 순서는 절대 바꾸지 않는 세션.

    플래너의 ``lookahead``와 혼동하면 안 된다. 그쪽은 k개 중 점수가 제일
    좋은 것을 **먼저 놓는다** — 벤치마크에서는 정당하지만 우리 설비에서는
    실행할 수 없다. 벨트 위 세 번째 박스를 앞의 둘을 건너뛰고 집을 수 없기
    때문이다.

    여기서 미래 정보는 **현재 박스를 어디에 놓을지 고르는 데에만** 쓴다:

    1. 현재 박스의 후보를 휴리스틱 점수로 정렬해 상위 ``beam_width``개만 남긴다
    2. 각 후보마다 거기 놓았다고 치고, 다음 박스들을 그리디로 이어 놓아본다
    3. 가장 많이 받아낸 후보를 고른다
    4. **현재 박스만** 실제로 놓는다. 나머지 롤아웃은 버린다

    ``upcoming``이 비면 후보 순위가 그대로 남아 기본 세션과 같은 결과가 된다.
    """

    def __init__(self, planner: DeepPack3DPlanner, *, beam_width: int = 3) -> None:
        super().__init__(planner)
        if beam_width < 1:
            raise ValueError(f"beam_width must be at least 1: {beam_width}")
        self.beam_width = beam_width

    def place(
        self, item: PackingItem, upcoming: Sequence[PackingItem] = ()
    ) -> PackingPlacement | None:
        """현재 박스를 놓는다. ``upcoming``은 뒤따라올 박스들(놓지는 않는다)."""
        if any(p.item.item_id == item.item_id for p in self.placements):
            raise ValueError(f"이미 배치된 item_id입니다: {item.item_id}")

        pending = [(self._offered, item)]
        candidates = self.planner._candidates(self._partitioner, pending)
        self._offered += 1
        if not candidates:
            return None

        ranked = sorted(candidates, key=self.planner._score)[: self.beam_width]
        if upcoming and len(ranked) > 1:
            selected = min(ranked, key=lambda c: self._rollout_score(c, upcoming))
        else:
            selected = ranked[0]
        return self.planner._apply_candidate(
            self._partitioner, selected, self.placements
        )

    def _rollout_score(
        self, candidate: _Candidate, upcoming: Sequence[PackingItem]
    ) -> tuple:
        """후보 하나를 놓고 남은 박스를 그리디로 이어 놓아본 결과를 점수화한다.

        작을수록 좋다. 순서대로 (못 받아낸 개수, 쌓인 높이 합, 원래 휴리스틱
        점수)다. 마지막 항이 있어야 롤아웃이 비겼을 때 기본 세션과 같은 답이
        나온다.
        """
        clone = _clone_partitioner(self._partitioner, self.planner.grid_size)
        scratch = list(self.placements)
        self.planner._apply_candidate(clone, candidate, scratch)

        placed = 0
        for offset, future in enumerate(upcoming, start=1):
            pending = [(self._offered + offset, future)]
            if self.planner._place_one(clone, pending, scratch) is None:
                # 벨트는 한 줄이라 하나가 막히면 뒤도 못 들어온다.
                break
            placed += 1

        return (
            len(upcoming) - placed,
            int(clone.height_map.sum()),
            self.planner._score(candidate),
        )


class StableDeepPack3DPlanner(DeepPack3DPlanner):
    """수직 지지만 보는 기본 플래너에 수평 지지 + 무게중심 필터를 더한다.

    기본 플래너의 ``min_support_ratio``는 "발자국의 절반 넘게 받쳐지는가"만
    본다. 그래서 지지가 한쪽으로 몰려 무게중심이 지지영역 밖으로 나가는
    배치도 통과한다 — 격자 위에서는 합법이지만 실물은 넘어진다.

    후보 필터 한 곳(`_candidates`)만 좁히므로 bl/baf/bssf/blsf 선택 로직은
    건드리지 않는다. 임계값을 0으로 두면 기본 플래너와 완전히 같아진다.

    바닥층(``z == 0``)은 팔레트가 발자국 전체를 통짜로 받치므로 검사에서
    제외한다. 높이 맵이 0이라 지지 셀 판정이 무의미하기 때문이다.
    """

    def __init__(
        self,
        *args,
        min_horizontal_support_ratio: float = 0.3,
        min_com_margin_ratio: float = 0.05,
        stability_tiebreak: bool = False,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        if not 0.0 <= min_horizontal_support_ratio <= 1.0:
            raise ValueError(
                "min_horizontal_support_ratio must be between 0 and 1: "
                f"{min_horizontal_support_ratio}"
            )
        if not -1.0 <= min_com_margin_ratio <= 1.0:
            raise ValueError(
                f"min_com_margin_ratio must be between -1 and 1: {min_com_margin_ratio}"
            )
        self.min_horizontal_support_ratio = min_horizontal_support_ratio
        self.min_com_margin_ratio = min_com_margin_ratio
        self.stability_tiebreak = stability_tiebreak
        self._stability: dict[int, tuple[float, float, float]] = {}

    def _candidates(
        self,
        partitioner: _SpacePartitioner,
        pending: list[tuple[int, PackingItem]],
    ) -> list[_Candidate]:
        accepted: list[_Candidate] = []
        self._stability = {}
        for candidate in super()._candidates(partitioner, pending):
            lateral = lateral_support_ratio(partitioner.occupied, candidate.cuboid)
            if lateral < self.min_horizontal_support_ratio:
                continue
            # 바닥층은 팔레트가 발자국 전체를 통짜로 받치므로 무게중심 검사에서
            # 제외한다. 높이 맵이 0이라 지지 셀 판정이 무의미하기 때문이다.
            com_margin = 0.5
            if candidate.cuboid.z > 0:
                _, com_margin = support_metrics(
                    partitioner.height_map, candidate.cuboid
                )
                if com_margin < self.min_com_margin_ratio:
                    continue
            edge_margin = support_edge_margin_ratio(
                partitioner.occupied, candidate.cuboid
            )
            self._stability[id(candidate)] = (com_margin, lateral, edge_margin)
            accepted.append(candidate)
        return accepted

    def _score(self, candidate: _Candidate) -> tuple[float, ...]:
        base = super()._score(candidate)
        com_margin, lateral, edge_margin = self._stability.get(
            id(candidate), (0.0, 0.0, 0.0)
        )
        if self.stability_tiebreak:
            # 휴리스틱 고유 점수는 그대로 보존하고, 기존 입력순서/원점
            # tie-breaker보다 COM/측면 지지만 먼저 비교한다.
            heuristic = base[:-5]
            identity = base[-5:]
            return heuristic + (-edge_margin, -com_margin, -lateral) + identity
        return base


def make_planner(
    pallet_aabb: Sequence[float],
    *,
    placer: str = "default",
    min_horizontal_support_ratio: float = 0.3,
    min_com_margin_ratio: float = 0.05,
    stability_tiebreak: bool = False,
    **kwargs,
) -> DeepPack3DPlanner:
    """``--placer`` 값에 맞는 플래너를 만든다.

    안정성 임계값은 ``placer="default"``일 때 조용히 무시된다. CLI가 항상
    세 값을 함께 넘기므로 호출부에서 분기하지 않아도 되게 하려는 것이다.
    """
    if placer not in SUPPORTED_PLACERS:
        raise ValueError(f"placer must be one of {SUPPORTED_PLACERS}: {placer}")
    if placer == "default":
        return DeepPack3DPlanner(pallet_aabb, **kwargs)
    return StableDeepPack3DPlanner(
        pallet_aabb,
        min_horizontal_support_ratio=min_horizontal_support_ratio,
        min_com_margin_ratio=min_com_margin_ratio,
        stability_tiebreak=stability_tiebreak,
        **kwargs,
    )


def compute_layer_slots(placements: Sequence[PackingPlacement]) -> list[tuple[int, int]]:
    """Map every placement to a (layer, slot) pair for the monitoring dashboard.

    Layers are the distinct bottom heights in ascending order, rounded to 1 mm so
    boxes resting on the same surface group together.  Within a layer the boxes
    are numbered by ``sequence_index``, i.e. the order the robot places them.

    The returned list is index-aligned with ``placements``.
    """
    if not placements:
        return []

    def level_of(placement: PackingPlacement) -> float:
        return round(placement.world_bottom_center[2], 3)

    levels = sorted({level_of(placement) for placement in placements})
    layer_of_level = {level: layer for layer, level in enumerate(levels)}

    result: list[tuple[int, int]] = [(0, 0)] * len(placements)
    next_slot: dict[int, int] = {}
    placement_order = sorted(
        range(len(placements)), key=lambda index: placements[index].sequence_index
    )
    for index in placement_order:
        layer = layer_of_level[level_of(placements[index])]
        slot = next_slot.get(layer, 0)
        next_slot[layer] = slot + 1
        result[index] = (layer, slot)
    return result
