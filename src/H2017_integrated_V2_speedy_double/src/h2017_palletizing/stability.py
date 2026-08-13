"""Pure release and settled-box stability checks.

The runtime supplies poses measured from USD/PhysX.  Keeping the acceptance
math here free of Isaac imports makes the sim-to-real thresholds unit-testable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class BoxPoseSample:
    center: tuple[float, float, float]
    up_axis: tuple[float, float, float]
    bottom_z: float


@dataclass(frozen=True)
class BoxStabilityResult:
    box_id: str
    horizontal_drift_m: float
    support_height_error_m: float
    tilt_deg: float
    passed: bool
    reasons: tuple[str, ...]


def is_drift_only_failure(
    result: BoxStabilityResult,
    *,
    trigger_drift_m: float,
    max_support_height_error_m: float,
    max_tilt_deg: float,
) -> bool:
    """Return whether only lateral drift exceeds the configured hard limit."""
    return (
        result.horizontal_drift_m > trigger_drift_m
        and abs(result.support_height_error_m) <= max_support_height_error_m
        and result.tilt_deg <= max_tilt_deg
    )


def box_tilt_degrees(up_axis) -> float:
    """Return the unsigned angle between the box local +Z and world +Z."""
    up = np.asarray(up_axis, dtype=np.float64)
    norm = float(np.linalg.norm(up))
    if not np.isfinite(norm) or norm < 1e-9:
        return math.inf
    cosine = float(np.clip(abs(up[2]) / norm, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def pose_motion_rates(previous: BoxPoseSample, current: BoxPoseSample, dt: float):
    """Return translational speed and box-up-axis angular speed between samples."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError("dt must be positive and finite")
    previous_center = np.asarray(previous.center, dtype=np.float64)
    current_center = np.asarray(current.center, dtype=np.float64)
    if previous_center.shape != (3,) or current_center.shape != (3,):
        raise ValueError("box centers must be 3-vectors")
    if not np.all(np.isfinite(previous_center)) or not np.all(np.isfinite(current_center)):
        raise ValueError("box centers must be finite")

    previous_up = np.asarray(previous.up_axis, dtype=np.float64)
    current_up = np.asarray(current.up_axis, dtype=np.float64)
    previous_norm = float(np.linalg.norm(previous_up))
    current_norm = float(np.linalg.norm(current_up))
    if previous_norm < 1e-9 or current_norm < 1e-9:
        return math.inf, math.inf
    cosine = float(np.clip(
        np.dot(previous_up, current_up) / (previous_norm * current_norm), -1.0, 1.0
    ))
    linear_speed = float(np.linalg.norm(current_center - previous_center) / dt)
    angular_speed = float(math.degrees(math.acos(cosine)) / dt)
    return linear_speed, angular_speed


def assess_box_pose(
    box_id: str,
    sample: BoxPoseSample,
    expected_center,
    box_height_m: float,
    *,
    max_horizontal_drift_m: float,
    max_support_height_error_m: float,
    max_tilt_deg: float,
) -> BoxStabilityResult:
    """Compare an observed rigid box pose with its planned support pose."""
    center = np.asarray(sample.center, dtype=np.float64)
    expected = np.asarray(expected_center, dtype=np.float64)
    if center.shape != (3,) or expected.shape != (3,):
        raise ValueError("box centers must be 3-vectors")
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(expected)):
        raise ValueError("box centers must be finite")
    if not math.isfinite(box_height_m) or box_height_m <= 0.0:
        raise ValueError("box height must be positive and finite")

    expected_support_z = float(expected[2] - box_height_m * 0.5)
    drift = float(np.linalg.norm(center[:2] - expected[:2]))
    support_error = float(sample.bottom_z - expected_support_z)
    tilt = box_tilt_degrees(sample.up_axis)

    reasons = []
    if drift > max_horizontal_drift_m:
        reasons.append(
            f"XY drift {drift * 1000.0:.1f} mm > "
            f"{max_horizontal_drift_m * 1000.0:.1f} mm"
        )
    if abs(support_error) > max_support_height_error_m:
        direction = "지지면 위 이격" if support_error > 0.0 else "지지면 관통/낙하"
        reasons.append(
            f"{direction} {abs(support_error) * 1000.0:.1f} mm > "
            f"{max_support_height_error_m * 1000.0:.1f} mm"
        )
    if tilt > max_tilt_deg:
        reasons.append(f"tilt {tilt:.2f}° > {max_tilt_deg:.2f}°")

    return BoxStabilityResult(
        box_id=box_id,
        horizontal_drift_m=drift,
        support_height_error_m=support_error,
        tilt_deg=tilt,
        passed=not reasons,
        reasons=tuple(reasons),
    )


def assess_release_pose(
    box_id: str,
    sample: BoxPoseSample,
    expected_center,
    box_height_m: float,
    *,
    min_gap_m: float,
    max_gap_m: float,
    max_horizontal_error_m: float,
    max_tilt_deg: float,
) -> BoxStabilityResult:
    """Check that a held box can safely transition from kinematic to dynamic."""
    center = np.asarray(sample.center, dtype=np.float64)
    expected = np.asarray(expected_center, dtype=np.float64)
    expected_support_z = float(expected[2] - box_height_m * 0.5)
    drift = float(np.linalg.norm(center[:2] - expected[:2]))
    gap = float(sample.bottom_z - expected_support_z)
    tilt = box_tilt_degrees(sample.up_axis)

    reasons = []
    if drift > max_horizontal_error_m:
        reasons.append(
            f"XY error {drift * 1000.0:.1f} mm > "
            f"{max_horizontal_error_m * 1000.0:.1f} mm"
        )
    if gap < min_gap_m:
        reasons.append(f"지지면 관통 {-gap * 1000.0:.1f} mm")
    elif gap > max_gap_m:
        reasons.append(
            f"릴리스 높이 {gap * 1000.0:.1f} mm > {max_gap_m * 1000.0:.1f} mm"
        )
    if tilt > max_tilt_deg:
        reasons.append(f"tilt {tilt:.2f}° > {max_tilt_deg:.2f}°")

    return BoxStabilityResult(
        box_id=box_id,
        horizontal_drift_m=drift,
        support_height_error_m=gap,
        tilt_deg=tilt,
        passed=not reasons,
        reasons=tuple(reasons),
    )
