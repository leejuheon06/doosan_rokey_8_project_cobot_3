"""Space-efficiency metrics for palletizing run summaries.

The standard volume efficiency and the project score are deliberately kept
separate.  The project score balances how much pallet floor is used with how
dense the occupied columns are, then applies the stability pass rate.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class SpaceEfficiency:
    footprint_coverage: float
    vertical_compactness: float
    stability_pass_rate: float
    project_efficiency: float
    occupied_surface_volume_m3: float


def calculate_space_efficiency(
    boxes: Iterable[tuple[Sequence[float], Sequence[float]]],
    pallet_aabb: Sequence[float],
    *,
    edge_margin_m: float = 0.0,
    stability_pass_count: int = 0,
    stability_check_count: int = 0,
) -> SpaceEfficiency:
    """Calculate exact axis-aligned footprint/column metrics.

    ``boxes`` contains ``(world_center, oriented_size)`` pairs.  X/Y boundary
    compression makes this exact for the planner's 0/90-degree placements;
    no raster resolution or empirical correction factor is involved.
    """
    pallet = tuple(float(value) for value in pallet_aabb)
    if len(pallet) != 6:
        raise ValueError("pallet_aabb must contain six values")
    if edge_margin_m < 0.0:
        raise ValueError("edge_margin_m must be non-negative")

    x_min = pallet[0] + edge_margin_m
    y_min = pallet[1] + edge_margin_m
    x_max = pallet[3] - edge_margin_m
    y_max = pallet[4] - edge_margin_m
    pallet_top = pallet[5]
    usable_area = max(x_max - x_min, 0.0) * max(y_max - y_min, 0.0)

    rectangles: list[tuple[float, float, float, float, float, float]] = []
    packed_volume = 0.0
    for center_values, size_values in boxes:
        center = tuple(float(value) for value in center_values)
        size = tuple(float(value) for value in size_values)
        if len(center) != 3 or len(size) != 3:
            raise ValueError("box center and size must each contain three values")
        if any(value <= 0.0 for value in size):
            continue
        left = max(center[0] - size[0] / 2.0, x_min)
        right = min(center[0] + size[0] / 2.0, x_max)
        back = max(center[1] - size[1] / 2.0, y_min)
        front = min(center[1] + size[1] / 2.0, y_max)
        if right <= left or front <= back:
            continue
        top = max(center[2] + size[2] / 2.0 - pallet_top, 0.0)
        rectangles.append((left, right, back, front, top, size[0] * size[1] * size[2]))
        packed_volume += size[0] * size[1] * size[2]

    footprint_area = 0.0
    occupied_surface_volume = 0.0
    if rectangles:
        xs = sorted({value for box in rectangles for value in box[:2]})
        ys = sorted({value for box in rectangles for value in box[2:4]})
        for xa, xb in zip(xs, xs[1:]):
            mid_x = (xa + xb) / 2.0
            for ya, yb in zip(ys, ys[1:]):
                mid_y = (ya + yb) / 2.0
                tops = [
                    box[4]
                    for box in rectangles
                    if box[0] <= mid_x < box[1] and box[2] <= mid_y < box[3]
                ]
                if not tops:
                    continue
                cell_area = (xb - xa) * (yb - ya)
                footprint_area += cell_area
                occupied_surface_volume += cell_area * max(tops)

    footprint = footprint_area / usable_area if usable_area > 0.0 else 0.0
    compactness = (
        packed_volume / occupied_surface_volume
        if occupied_surface_volume > 0.0 else 0.0
    )
    stability = (
        stability_pass_count / stability_check_count
        if stability_check_count > 0 else 0.0
    )
    # Floating-point and clipped-boundary noise must not produce >100% metrics.
    footprint = min(max(footprint, 0.0), 1.0)
    compactness = min(max(compactness, 0.0), 1.0)
    stability = min(max(stability, 0.0), 1.0)
    balanced_space = (
        2.0 * footprint * compactness / (footprint + compactness)
        if footprint + compactness > 0.0 else 0.0
    )
    return SpaceEfficiency(
        footprint_coverage=footprint,
        vertical_compactness=compactness,
        stability_pass_rate=stability,
        project_efficiency=balanced_space * stability,
        occupied_surface_volume_m3=occupied_surface_volume,
    )
