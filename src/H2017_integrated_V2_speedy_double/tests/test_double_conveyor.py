import numpy as np

from h2017_palletizing.config import (
    CONVEYOR_PICK_ZONE_MAXS,
    CONVEYOR_PICK_ZONE_MINS,
    CONVEYOR_RUN_SPEED_MPS,
    CONVEYOR_SPEED_ATTRIBUTE_PATHS,
    CONVEYOR_TEMPLATE_CUBE_PATHS,
    PICK_REGIONS,
)
from h2017_palletizing.coordination import has_reached_pick_zone


def test_two_lines_have_independent_configuration():
    assert CONVEYOR_RUN_SPEED_MPS == 1.2
    assert len(CONVEYOR_TEMPLATE_CUBE_PATHS) == 2
    assert len(CONVEYOR_SPEED_ATTRIBUTE_PATHS) == 2
    assert all(len(paths) == 3 for paths in CONVEYOR_SPEED_ATTRIBUTE_PATHS)
    assert len(set(PICK_REGIONS)) == 2


def test_second_pick_zone_accepts_only_second_belt():
    point = np.array([0.20, 1.95, 0.5])
    assert has_reached_pick_zone(
        point,
        CONVEYOR_PICK_ZONE_MINS[1],
        CONVEYOR_PICK_ZONE_MAXS[1],
    )
    assert not has_reached_pick_zone(
        point,
        CONVEYOR_PICK_ZONE_MINS[0],
        CONVEYOR_PICK_ZONE_MAXS[0],
    )


def test_pick_zone_accepts_box_that_passed_x_max():
    assert has_reached_pick_zone(
        np.array([0.50, 1.95, 0.5]),
        CONVEYOR_PICK_ZONE_MINS[1],
        CONVEYOR_PICK_ZONE_MAXS[1],
    )
