"""Pure coordination rules that can be tested without Isaac Sim."""

from __future__ import annotations

import numpy as np

from .config import PICK_ZONE_MAX, PICK_ZONE_MIN


class RegionLock:
    """Allow only one robot to own a shared work region at a time."""

    def __init__(self, names) -> None:
        self.owner: dict[str, str | None] = {name: None for name in names}

    def acquire(self, region: str, unit_name: str) -> bool:
        current = self.owner[region]
        if current is None:
            self.owner[region] = unit_name
            print(f"[LOCK] {region} <- {unit_name}")
            return True
        return current == unit_name

    def release(self, region: str, unit_name: str) -> None:
        if self.owner[region] == unit_name:
            self.owner[region] = None
            print(f"[LOCK] {region} 해제 ({unit_name})")

    def release_all(self, unit_name: str) -> None:
        for region in self.owner:
            self.release(region, unit_name)

    def reset(self) -> None:
        for region in self.owner:
            self.owner[region] = None


def rank_idle_unit_indices(units, sessions) -> list[int]:
    """Return idle robots in least-loaded-first order.

    Session placement counts directly describe how much of each robot's
    pallet half has been consumed.  Using them also prevents a fast first
    robot from starving every later robot once cycle time drops below the
    spawn interval.
    """
    return sorted(
        (index for index, unit in enumerate(units) if not unit.busy),
        key=lambda index: (len(sessions[index].placements), index),
    )


def conveyor_side_standby_target(
    cube_position,
    robot_base_position,
    max_box_height: float,
    gripper_offset: float,
    approach_height: float,
    lateral_offset: float,
) -> np.ndarray:
    """카메라와 컨베이어 경로 밖의 로봇별 측면 대기점을 계산한다."""
    cube = np.asarray(cube_position, dtype=np.float64)
    base = np.asarray(robot_base_position, dtype=np.float64)
    side = 1.0 if base[1] >= cube[1] else -1.0
    return np.array(
        [
            cube[0],
            cube[1] + side * float(lateral_offset),
            cube[2]
            + float(max_box_height) * 0.5
            + float(gripper_offset)
            + float(approach_height),
        ],
        dtype=np.float64,
    )


def has_reached_pick_zone(
    position,
    pick_zone_min=PICK_ZONE_MIN,
    pick_zone_max=PICK_ZONE_MAX,
) -> bool:
    """Return true once a forward-moving box reaches or passes the pick zone."""
    position = np.asarray(position, dtype=np.float64)
    pick_zone_min = np.asarray(pick_zone_min, dtype=np.float64)
    pick_zone_max = np.asarray(pick_zone_max, dtype=np.float64)
    return bool(
        position[0] >= pick_zone_min[0]
        and np.all(position[1:] >= pick_zone_min[1:])
        and np.all(position[1:] <= pick_zone_max[1:])
    )


def footprint_intersects_center_interlock(
    center_y: float,
    footprint_depth: float,
    pallet_mid_y: float,
    interlock_half_width: float,
) -> bool:
    """Return whether a placement footprint enters the shared center band."""
    if footprint_depth <= 0 or interlock_half_width < 0:
        raise ValueError("footprint_depth must be positive and interlock width non-negative")
    nearest_edge_distance = abs(float(center_y) - float(pallet_mid_y)) - footprint_depth * 0.5
    return nearest_edge_distance <= float(interlock_half_width)


def center_interlock_staging_box_y(
    robot_base_y: float,
    pallet_mid_y: float,
    footprint_depth: float,
    interlock_half_width: float,
    clearance: float,
) -> float:
    """Return a payload-center Y that stays fully outside the center band."""
    if footprint_depth <= 0 or interlock_half_width < 0 or clearance < 0:
        raise ValueError("footprint depth, interlock width and clearance are invalid")
    side = 1.0 if float(robot_base_y) >= float(pallet_mid_y) else -1.0
    offset = (
        float(interlock_half_width)
        + float(footprint_depth) * 0.5
        + float(clearance)
    )
    return float(pallet_mid_y) + side * offset


def quaternion_angular_error_deg(current, target) -> float:
    """Return the shortest angular distance between scalar-first quaternions."""
    current = np.asarray(current, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    if current.shape != (4,) or target.shape != (4,):
        raise ValueError("quaternions must be 4-vectors")
    current_norm = float(np.linalg.norm(current))
    target_norm = float(np.linalg.norm(target))
    if current_norm < 1e-12 or target_norm < 1e-12:
        raise ValueError("quaternions must have non-zero norm")
    dot = abs(float(np.dot(current / current_norm, target / target_norm)))
    return float(np.degrees(2.0 * np.arccos(np.clip(dot, -1.0, 1.0))))
