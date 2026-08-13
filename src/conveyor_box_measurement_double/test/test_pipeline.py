import numpy as np
from pathlib import Path

from conveyor_box_measurement.node import (
    BoxMeasurement,
    PipelineConfig,
    consume_empty_detection_retry,
    depth_to_pointcloud,
    estimate_reference_depth,
    is_plausible_box_size,
    quaternion_to_rotation_matrix,
    select_measurement_nearest_camera_axis,
    with_default_params_file,
)


def test_default_params_file_is_added_to_plain_command():
    params_file = Path("/tmp/measurement.yaml")
    assert with_default_params_file(["node"], params_file) == [
        "node",
        "--ros-args",
        "--params-file",
        str(params_file),
    ]


def test_explicit_params_file_is_not_replaced():
    args = ["node", "--ros-args", "--params-file", "/tmp/custom.yaml"]
    assert with_default_params_file(args, Path("/tmp/default.yaml")) == args


def test_cli_parameter_override_follows_default_yaml():
    args = ["node", "--ros-args", "-p", "publish_debug:=false"]
    assert with_default_params_file(args, Path("/tmp/default.yaml")) == [
        "node",
        "--ros-args",
        "--params-file",
        "/tmp/default.yaml",
        "-p",
        "publish_debug:=false",
    ]


def test_empty_detection_consumes_retry_budget():
    assert consume_empty_detection_retry(False, 2) == (True, 1)
    assert consume_empty_detection_retry(False, 1) == (True, 0)
    assert consume_empty_detection_retry(False, 0) == (False, 0)


def test_detection_finishes_without_consuming_retry_budget():
    assert consume_empty_detection_retry(True, 2) == (False, 2)


def test_depth_to_pointcloud_filters_invalid_depth():
    depth = np.array([[1.0, 0.0], [np.nan, 2.0]], dtype=np.float32)
    points = depth_to_pointcloud(depth, fx=1.0, fy=1.0, cx=0.0, cy=0.0)
    np.testing.assert_allclose(points, [[0.0, 0.0, 1.0], [2.0, 2.0, 2.0]])


def test_reference_depth_uses_configured_percentile():
    points = np.array([[0.0, 0.0, z] for z in (1.0, 2.0, 3.0, 4.0)])
    config = PipelineConfig(reference_depth_percentile=50.0)
    assert estimate_reference_depth(points, config) == 2.5


def test_identity_quaternion_produces_identity_matrix():
    np.testing.assert_allclose(
        quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0), np.eye(3)
    )


def test_selects_candidate_nearest_camera_axis_before_world_transform():
    box = BoxMeasurement(
        cluster_id=1,
        center_xyz_m=np.array([0.01, -0.02, 0.80]),
        size_xyz_m=np.array([0.31, 0.41, 0.28]),
        yaw_rad=0.0,
        num_points=1000,
    )
    robot_link = BoxMeasurement(
        cluster_id=2,
        center_xyz_m=np.array([0.15, 0.10, 0.50]),
        size_xyz_m=np.array([0.14, 0.13, 0.34]),
        yaw_rad=0.0,
        num_points=1000,
    )

    assert select_measurement_nearest_camera_axis([robot_link, box]) is box


def test_select_returns_none_when_there_are_no_candidates():
    assert select_measurement_nearest_camera_axis([]) is None


def test_plausible_box_filter_keeps_catalog_boxes_and_rejects_robot_links():
    config = PipelineConfig()
    assert is_plausible_box_size(np.array([0.1888, 0.2187, 0.09]), config)
    assert is_plausible_box_size(np.array([0.3096, 0.4077, 0.28]), config)
    assert not is_plausible_box_size(np.array([0.10, 0.12, 0.85]), config)
    assert not is_plausible_box_size(np.array([0.04, 0.02, 0.21]), config)
