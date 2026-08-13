#!/usr/bin/env python3
"""
컨베이어 위 박스 1개의 중심 좌표와 측정 크기를 추정하는 ROS 2 비전 노드.

입력:
  - depth image (`sensor_msgs/Image`)
  - camera info (`sensor_msgs/CameraInfo`)
  - conveyor status (`std_msgs/Bool`, `True`일 때 1회 측정)

출력:
  - `/vision/box_detections` (`std_msgs/String`)

처리 순서:
  1. depth를 point cloud로 역투영
  2. 기준 평면을 제거
  3. 남은 점군을 클러스터링
  4. 박스 중심, 크기, yaw를 추정
  5. 여러 박스가 검출되면 프레임 중심에 가장 가까운 박스 1개만 선택
  6. 선택된 박스를 JSON으로 발행

출력 JSON 예시:
  {
    "stamp": 1786174308,
    "box": {
      "id": 3,
      "center_m": [0.2374, -0.7113, 0.7872],
      "size_m": [0.1982, 0.1494, 0.1500],
      "yaw_rad": -1.5708
    }
  }
"""

from __future__ import annotations

import dataclasses
import json
import sys
from pathlib import Path
from typing import List, Optional

import numpy as np

# ROS Humble의 tf_transformations가 의존하는 transforms3d는 구버전일 수 있어
# 최신 NumPy에서 제거된 np.float 별칭을 기대한다.
if not hasattr(np, "float"):
    np.float = float  # type: ignore[attr-defined]

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    import open3d as o3d
except ImportError:  # pragma: no cover
    o3d = None

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from ament_index_python.packages import get_package_share_directory


def with_default_params_file(args: List[str], params_file: Path) -> List[str]:
    """Add the package YAML unless the caller already selected a params file."""
    if any(
        arg == "--params-file" or arg.startswith("--params-file=") for arg in args
    ):
        return args

    result = list(args)
    default_params_args = ["--params-file", str(params_file)]
    try:
        ros_args_index = result.index("--ros-args")
    except ValueError:
        result.extend(["--ros-args", *default_params_args])
    else:
        # 기본 YAML을 먼저 읽고, 뒤에 오는 사용자의 -p 오버라이드를 우선한다.
        insert_at = ros_args_index + 1
        result[insert_at:insert_at] = default_params_args
    return result


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """단위 쿼터니언을 3x3 회전 행렬로 바꾼다.

    이 한 함수 때문에 tf_transformations(→transforms3d)를 의존성으로 들이지
    않는다. transforms3d 구버전은 NumPy에서 제거된 np.float 별칭을 기대해
    설치 자체가 골칫거리다.
    """
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm == 0.0:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    CvBridge = None


# --------------------------------------------------------------------------
# 구성 파라미터 / 측정 결과 구조체
# --------------------------------------------------------------------------

@dataclasses.dataclass
class PipelineConfig:
    plane_distance_threshold_m: float = 0.006
    cluster_eps_m: float = 0.02
    cluster_min_points: int = 30
    min_box_height_m: float = 0.05
    # H2017 입고 박스(1~4호: 최소 바닥면 0.19 x 0.22 m, 최대 높이
    # 0.28 m)보다 충분히 넓은 범위다. 카메라 ROI에 들어온 로봇 링크처럼
    # 지나치게 얇거나 높은 클러스터만 제거한다.
    min_box_footprint_side_m: float = 0.12
    max_box_footprint_side_m: float = 0.50
    max_box_height_m: float = 0.35
    # 기준 평면(벨트) 깊이를 잡을 depth 백분위수.
    #
    # 예전에는 RANSAC으로 "가장 큰 평면"을 벨트라고 가정했는데, 큰 박스에서
    # 그 가정이 깨진다. 우체국 4호(0.41 x 0.31)의 윗면은 ROI 면적의 약 67%를
    # 차지해 벨트보다 큰 평면이 된다. 그러면 RANSAC이 박스 윗면을 기준면으로
    # 골라 높이가 음수로 나오고 박스가 통째로 버려진다(실측: 4호 3개 전부 실패).
    #
    # 탑뷰 카메라에서는 벨트가 항상 가장 먼 면이므로, 깊이 분포의 높은
    # 백분위수를 잡으면 박스 크기와 무관하게 벨트가 나온다. 박스가 ROI의 67%를
    # 덮어도 벨트는 상위 33%에 남으므로 85는 안전한 값이다.
    reference_depth_percentile: float = 85.0


def consume_empty_detection_retry(
    has_detection: bool, retries_remaining: int
) -> tuple[bool, int]:
    """Return whether an empty frame should be retried and the new budget."""
    if retries_remaining < 0:
        raise ValueError("retries_remaining must be non-negative")
    if has_detection or retries_remaining == 0:
        return False, retries_remaining
    return True, retries_remaining - 1


@dataclasses.dataclass
class BoxMeasurement:
    cluster_id: int
    center_xyz_m: np.ndarray
    size_xyz_m: np.ndarray
    yaw_rad: float
    num_points: int
    box_id: int = -1

    def to_dict(self) -> dict:
        return {
            "id": self.box_id if self.box_id >= 0 else self.cluster_id,
            "center_m": [float(v) for v in self.center_xyz_m],
            "size_m": [float(v) for v in self.size_xyz_m],
            "yaw_rad": float(self.yaw_rad),
        }


@dataclasses.dataclass
class DetectionDebug:
    """한 프레임에서 RViz 시각화에 필요한 중간 결과."""

    points_cam: Optional[np.ndarray] = None
    raised_points: Optional[np.ndarray] = None
    clusters: Optional[List[np.ndarray]] = None
    reference_z_m: Optional[float] = None


# --------------------------------------------------------------------------
# 순수 처리 함수
# --------------------------------------------------------------------------

def depth_to_pointcloud(
    depth_map_m: np.ndarray, fx: float, fy: float, cx: float, cy: float
) -> np.ndarray:
    """유효 depth만 사용해 카메라 좌표계 point cloud를 생성한다."""
    h, w = depth_map_m.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    z = depth_map_m
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    # 0 / NaN / Inf depth는 유효 점으로 사용하지 않는다.
    valid = np.isfinite(z.reshape(-1)) & (z.reshape(-1) > 0)
    return points[valid]


def estimate_reference_depth(points: np.ndarray, cfg: PipelineConfig) -> float:
    """벨트 상면의 카메라 깊이를 추정한다.

    탑뷰이므로 벨트는 ROI 안에서 가장 먼 면이다. 큰 박스가 시야를 대부분
    채워도 가장자리에 벨트가 남으므로, 높은 백분위수를 잡으면 박스 크기와
    무관하게 벨트가 나온다. RANSAC "최대 평면" 가정과 달리 여기서는 박스가
    커질수록 오히려 안전해진다.
    """
    return float(np.percentile(points[:, 2], cfg.reference_depth_percentile))


def points_above_reference(points: np.ndarray, reference_z: float, cfg: PipelineConfig):
    """기준 평면보다 카메라 쪽으로 솟은 점만 남긴다."""
    if o3d is None:
        raise RuntimeError("open3d가 필요합니다: pip install open3d --break-system-packages")

    mask = points[:, 2] < reference_z - cfg.plane_distance_threshold_m
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[mask])
    return pcd


def cluster_boxes(remaining_pcd, cfg: PipelineConfig) -> List[np.ndarray]:
    if len(remaining_pcd.points) == 0:
        return []

    labels = np.array(
        remaining_pcd.cluster_dbscan(
            eps=cfg.cluster_eps_m, min_points=cfg.cluster_min_points
        )
    )
    points = np.asarray(remaining_pcd.points)

    clusters = []
    for cluster_id in range(labels.max() + 1 if labels.size else 0):
        mask = labels == cluster_id
        if mask.sum() < cfg.cluster_min_points:
            continue
        clusters.append(points[mask])
    return clusters


def is_plausible_box_size(size_xyz_m: np.ndarray, cfg: PipelineConfig) -> bool:
    """Reject geometry that is clearly outside the configured parcel envelope."""
    width, length, height = (float(value) for value in size_xyz_m)
    return bool(
        min(width, length) >= cfg.min_box_footprint_side_m
        and max(width, length) <= cfg.max_box_footprint_side_m
        and cfg.min_box_height_m <= height <= cfg.max_box_height_m
    )


def measure_box_from_cluster(
    cluster_id: int, cluster_points: np.ndarray, plane_z_m: float, cfg: PipelineConfig
) -> Optional[BoxMeasurement]:
    if cv2 is None:
        raise RuntimeError("opencv-python이 필요합니다: pip install opencv-python --break-system-packages")

    top_view_xy = cluster_points[:, :2].astype(np.float32)
    (cx, cy), (w, l), angle_deg = cv2.minAreaRect(top_view_xy)

    # 탑뷰 카메라를 가정하므로, 박스 윗면의 depth가 기준 평면 depth보다 작다.
    # 따라서 박스 높이는 기준 평면 z와 박스 윗면 최소 z의 차이로 계산한다.
    z_box_top = float(cluster_points[:, 2].min())
    height_m = plane_z_m - z_box_top
    size_xyz_m = np.array([w, l, height_m], dtype=np.float32)
    if not is_plausible_box_size(size_xyz_m, cfg):
        return None

    z_center = z_box_top + height_m / 2.0

    return BoxMeasurement(
        cluster_id=cluster_id,
        center_xyz_m=np.array([cx, cy, z_center], dtype=np.float32),
        size_xyz_m=size_xyz_m,
        yaw_rad=np.deg2rad(angle_deg),
        num_points=cluster_points.shape[0],
    )


def select_measurement_nearest_camera_axis(
    measurements: List[BoxMeasurement],
) -> Optional[BoxMeasurement]:
    """카메라 좌표의 광축(x=0, y=0)에 가장 가까운 박스를 고른다.

    반드시 World 좌표 변환 전에 호출해야 한다. World XY 원점 거리를 쓰면
    프레임 중심과 무관하게 로봇 링크가 선택될 수 있다.
    """
    if not measurements:
        return None
    return min(
        measurements,
        key=lambda measurement: float(np.linalg.norm(measurement.center_xyz_m[:2])),
    )


def run_detection_pipeline(
    depth_map_m: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    cfg: PipelineConfig,
    log=None,
    debug: Optional[DetectionDebug] = None,
) -> List[BoxMeasurement]:
    """depth 한 장에서 박스 후보들을 추정한다.

    log가 주어지면 단계별 중간값을 넘긴다. 검출 실패는 원인이 여러 갈래라
    (점군 없음 / 솟은 점 없음 / 클러스터 없음 / 높이 미달) 로그 없이는
    구분되지 않는다.
    """
    def note(message: str) -> None:
        if log is not None:
            log(message)

    points_cam = depth_to_pointcloud(depth_map_m, fx, fy, cx, cy)
    if debug is not None:
        debug.points_cam = points_cam
    if points_cam.shape[0] == 0:
        note("유효 depth 점 없음")
        return []

    reference_z = estimate_reference_depth(points_cam, cfg)
    if debug is not None:
        debug.reference_z_m = reference_z
    remaining_pcd = points_above_reference(points_cam, reference_z, cfg)
    if debug is not None:
        debug.raised_points = np.asarray(remaining_pcd.points)
    raised = len(remaining_pcd.points)
    note(
        f"점 {points_cam.shape[0]}개, 기준면 깊이 {reference_z:.4f} m, "
        f"솟은 점 {raised}개"
    )
    if raised == 0:
        return []

    clusters = cluster_boxes(remaining_pcd, cfg)
    if debug is not None:
        debug.clusters = clusters
    note(f"클러스터 {len(clusters)}개")

    measurements = []
    for cid, cluster_points in enumerate(clusters):
        result = measure_box_from_cluster(cid, cluster_points, reference_z, cfg)
        if result is None:
            height = reference_z - float(cluster_points[:, 2].min())
            top_view_xy = cluster_points[:, :2].astype(np.float32)
            (_, _), (width, length), _ = cv2.minAreaRect(top_view_xy)
            note(
                f"  클러스터 {cid}: 점 {cluster_points.shape[0]}개, "
                f"크기 {[round(float(width), 4), round(float(length), 4), round(height, 4)]} m "
                "— 박스 범위 밖, 버림"
            )
            continue
        note(
            f"  클러스터 {cid}: 크기 {np.round(result.size_xyz_m, 4).tolist()} m"
        )
        measurements.append(result)
    return measurements


# --------------------------------------------------------------------------
# ROS 2 노드
# --------------------------------------------------------------------------

class ConveyorBoxMeasurementNode(Node):
    def __init__(self) -> None:
        super().__init__("conveyor_box_measurement_node")

        # 런치 파일 또는 CLI에서 교체 가능한 입력/출력 및 처리 파라미터.
        self.declare_parameter("depth_topic", "/cv_depth")
        self.declare_parameter("camera_info_topic", "/cv_camera_info")
        self.declare_parameter("conveyor_topic", "/conveyor/status")
        self.declare_parameter("detections_topic", "/vision/box_detections")
        self.declare_parameter("output_frame", "World")
        self.declare_parameter("use_depth_roi", True)
        # ROI는 1280x720 기준이다. u(roi_x)가 벨트 폭 방향, v(roi_y)가 진행 방향.
        #
        # 박스 윗면은 벨트보다 박스 높이만큼 카메라에 가까워 더 크게 투영된다.
        # 4호(0.41 x 0.31 x 0.28) 윗면은 u 481~724, v 146~434를 차지하는데,
        # 이전 값(x 510~770)은 폭 방향으로 29 px을 잘라 폭을 0.31 대신 0.273으로
        # 재게 만들었다. 그러면 3호로 오분류된다.
        #
        # roi_x_min=470은 근측 가드레일을 11 px 물지만, 481로 하면 박스 여유가 0이라
        # 8.8 cm 낙하 후 조금만 밀려도 잘린다. 레일은 ROI의 2% 미만이라 RANSAC
        # 평면 선택을 흔들지 못한다.
        #
        # roi_y_max를 480에서 더 못 넓히는 이유는 벨트 끝(x=0.5)이 v=490에 오기
        # 때문이다. 그 너머는 바닥이라 RANSAC이 기준 평면을 잘못 고를 수 있다.
        self.declare_parameter("roi_x_min_px", 470)
        self.declare_parameter("roi_y_min_px", 120)
        self.declare_parameter("roi_x_max_px", 760)
        self.declare_parameter("roi_y_max_px", 480)
        self.declare_parameter("plane_distance_threshold_m", 0.006)
        self.declare_parameter("cluster_eps_m", 0.02)
        self.declare_parameter("cluster_min_points", 30)
        self.declare_parameter("min_box_height_m", 0.05)
        self.declare_parameter("min_box_footprint_side_m", 0.12)
        self.declare_parameter("max_box_footprint_side_m", 0.50)
        self.declare_parameter("max_box_height_m", 0.35)
        self.declare_parameter("reference_depth_percentile", 85.0)
        self.declare_parameter("empty_detection_retry_frames", 2)
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("debug_publish_rate_hz", 5.0)
        self.declare_parameter("debug_point_stride", 4)
        self.declare_parameter("debug_depth_topic", "/vision/debug/depth_image")
        self.declare_parameter("debug_overlay_topic", "/vision/debug/overlay_image")
        self.declare_parameter("debug_pointcloud_topic", "/vision/debug/pointcloud")
        self.declare_parameter("debug_raised_points_topic", "/vision/debug/raised_points")
        self.declare_parameter("debug_markers_topic", "/vision/debug/markers")
        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        conveyor_topic = self.get_parameter("conveyor_topic").value
        detections_topic = self.get_parameter("detections_topic").value
        self._output_frame = self.get_parameter("output_frame").value

        self._cfg = PipelineConfig(
            plane_distance_threshold_m=self.get_parameter("plane_distance_threshold_m").value,
            cluster_eps_m=self.get_parameter("cluster_eps_m").value,
            cluster_min_points=self.get_parameter("cluster_min_points").value,
            min_box_height_m=self.get_parameter("min_box_height_m").value,
            min_box_footprint_side_m=self.get_parameter(
                "min_box_footprint_side_m"
            ).value,
            max_box_footprint_side_m=self.get_parameter(
                "max_box_footprint_side_m"
            ).value,
            max_box_height_m=self.get_parameter("max_box_height_m").value,
            reference_depth_percentile=self.get_parameter(
                "reference_depth_percentile"
            ).value,
        )

        # intrinsics가 준비된 뒤에만 backprojection이 가능하다.
        self._intrinsics: Optional[dict] = None
        # 노드가 살아 있는 동안 발행 순서대로 증가하는 박스 번호.
        self._next_box_id: int = 0
        # conveyor/status가 True일 때 새 depth 프레임을 처리한다. 빈 프레임이면
        # 설정된 예산만큼 후속 프레임을 더 확인한다.
        self._pending_trigger: bool = False
        self._empty_detection_retries_remaining: int = 0
        self._last_debug_publish_ns: int = 0
        self._bridge = CvBridge() if CvBridge is not None else None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # 센서 스트림은 최신 프레임 위주로 처리한다.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, sensor_qos)
        self.create_subscription(Bool, conveyor_topic, self._on_conveyor_status, 10)

        self._detections_pub = self.create_publisher(String, detections_topic, 10)
        self._debug_depth_pub = self.create_publisher(
            Image, self.get_parameter("debug_depth_topic").value, sensor_qos
        )
        self._debug_overlay_pub = self.create_publisher(
            Image, self.get_parameter("debug_overlay_topic").value, sensor_qos
        )
        self._debug_pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("debug_pointcloud_topic").value,
            sensor_qos,
        )
        self._debug_raised_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("debug_raised_points_topic").value,
            sensor_qos,
        )
        self._debug_markers_pub = self.create_publisher(
            MarkerArray, self.get_parameter("debug_markers_topic").value, 10
        )

        self.get_logger().info(
            f"conveyor_box_measurement_node 시작 | depth='{depth_topic}' info='{camera_info_topic}' "
            f"conveyor='{conveyor_topic}' output_frame='{self._output_frame}' "
            f"-> publish '{detections_topic}'"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        # sensor_msgs/CameraInfo.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        k = msg.k if hasattr(msg, "k") else msg.K  # ROS2 rolling: k(소문자) / 일부 버전 K
        self._intrinsics = {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}

    def _on_conveyor_status(self, msg: Bool) -> None:
        self._pending_trigger = bool(msg.data)
        if self._pending_trigger:
            self._empty_detection_retries_remaining = max(
                0, int(self.get_parameter("empty_detection_retry_frames").value)
            )
        else:
            self._empty_detection_retries_remaining = 0

    def _on_depth(self, msg: Image) -> None:
        if self._intrinsics is None:
            # intrinsic 없이는 depth를 3D 점군으로 바꿀 수 없다.
            return

        full_depth_m = self._convert_depth_image(msg)
        if full_depth_m is None:
            return
        depth_m = self._apply_depth_roi(full_depth_m)

        triggered = bool(getattr(self, "_pending_trigger", False))
        debug_due = self._debug_publish_due()
        if not triggered and not debug_due:
            return
        debug = DetectionDebug() if debug_due else None
        measurements = run_detection_pipeline(
            depth_m,
            self._intrinsics["fx"],
            self._intrinsics["fy"],
            self._intrinsics["cx"],
            self._intrinsics["cy"],
            self._cfg,
            log=self.get_logger().info if triggered else None,
            debug=debug,
        )

        if debug_due and debug is not None:
            self._publish_debug(msg, full_depth_m, measurements, debug)

        if not triggered:
            return

        retry, remaining = consume_empty_detection_retry(
            bool(measurements), self._empty_detection_retries_remaining
        )
        self._empty_detection_retries_remaining = remaining
        if retry:
            self._pending_trigger = True
            self.get_logger().warn(
                "empty detection frame; waiting for a newer depth frame "
                f"({remaining} retries remaining)"
            )
            return
        self._pending_trigger = False

        selected_measurement = select_measurement_nearest_camera_axis(measurements)
        if len(measurements) > 1:
            self.get_logger().warn(
                f"multiple plausible boxes detected ({len(measurements)}); "
                "selected the box closest to the camera axis",
                throttle_duration_sec=2.0,
            )
        selected_measurements = (
            [selected_measurement] if selected_measurement is not None else []
        )
        frame_id = self._transform_measurements_to_output_frame(
            selected_measurements,
            source_frame=msg.header.frame_id,
            stamp=msg.header.stamp,
        )
        self._publish_detections(selected_measurement, frame_id)

    def _debug_publish_due(self) -> bool:
        if not bool(self.get_parameter("publish_debug").value):
            return False
        rate_hz = max(0.1, float(self.get_parameter("debug_publish_rate_hz").value))
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_debug_publish_ns < int(1e9 / rate_hz):
            return False
        self._last_debug_publish_ns = now_ns
        return True

    def _publish_debug(
        self,
        source_msg: Image,
        full_depth_m: np.ndarray,
        measurements: List[BoxMeasurement],
        debug: DetectionDebug,
    ) -> None:
        if self._bridge is not None:
            depth_message = self._bridge.cv2_to_imgmsg(full_depth_m, encoding="32FC1")
            depth_message.header = source_msg.header
            self._debug_depth_pub.publish(depth_message)

            overlay = self._make_debug_overlay(full_depth_m, measurements)
            overlay_message = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_message.header = source_msg.header
            self._debug_overlay_pub.publish(overlay_message)

        stride = max(1, int(self.get_parameter("debug_point_stride").value))
        if debug.points_cam is not None:
            cloud = point_cloud2.create_cloud_xyz32(
                source_msg.header, debug.points_cam[::stride].astype(np.float32).tolist()
            )
            self._debug_pointcloud_pub.publish(cloud)
        if debug.raised_points is not None:
            raised = point_cloud2.create_cloud_xyz32(
                source_msg.header, debug.raised_points[::stride].astype(np.float32).tolist()
            )
            self._debug_raised_pub.publish(raised)

        self._debug_markers_pub.publish(
            self._make_debug_markers(source_msg, measurements, debug)
        )

    def _make_debug_overlay(
        self, depth_m: np.ndarray, measurements: List[BoxMeasurement]
    ) -> np.ndarray:
        valid = depth_m[np.isfinite(depth_m) & (depth_m > 0)]
        if valid.size == 0 or cv2 is None:
            return np.zeros((*depth_m.shape, 3), dtype=np.uint8)

        near, far = np.percentile(valid, [2.0, 98.0])
        span = max(float(far - near), 1e-6)
        normalized = np.clip((depth_m - near) * 255.0 / span, 0, 255).astype(np.uint8)
        overlay = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_TURBO)
        overlay[depth_m <= 0] = 0

        x_min = int(self.get_parameter("roi_x_min_px").value)
        y_min = int(self.get_parameter("roi_y_min_px").value)
        x_max = int(self.get_parameter("roi_x_max_px").value)
        y_max = int(self.get_parameter("roi_y_max_px").value)
        cv2.rectangle(overlay, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        fx, fy = self._intrinsics["fx"], self._intrinsics["fy"]
        cx, cy = self._intrinsics["cx"], self._intrinsics["cy"]
        for measurement in measurements:
            center_x, center_y, center_z = measurement.center_xyz_m
            width, length, height = measurement.size_xyz_m
            top_z = max(float(center_z - height / 2.0), 1e-6)
            local = np.array(
                [
                    [-width / 2.0, -length / 2.0],
                    [width / 2.0, -length / 2.0],
                    [width / 2.0, length / 2.0],
                    [-width / 2.0, length / 2.0],
                ]
            )
            cosine, sine = np.cos(measurement.yaw_rad), np.sin(measurement.yaw_rad)
            rotation = np.array([[cosine, -sine], [sine, cosine]])
            corners = local @ rotation.T + np.array([center_x, center_y])
            pixels = np.column_stack(
                (fx * corners[:, 0] / top_z + cx, fy * corners[:, 1] / top_z + cy)
            ).astype(np.int32)
            cv2.polylines(overlay, [pixels], True, (0, 255, 255), 2)
            anchor = tuple(pixels[0])
            label = f"{width:.3f} x {length:.3f} x {height:.3f} m"
            cv2.putText(overlay, label, anchor, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        return overlay

    def _make_debug_markers(
        self,
        source_msg: Image,
        measurements: List[BoxMeasurement],
        debug: DetectionDebug,
    ) -> MarkerArray:
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if debug.points_cam is not None and debug.points_cam.size and debug.reference_z_m is not None:
            plane = Marker()
            plane.header = source_msg.header
            plane.ns = "reference_plane"
            plane.id = 0
            plane.type = Marker.CUBE
            plane.action = Marker.ADD
            mins = debug.points_cam[:, :2].min(axis=0)
            maxs = debug.points_cam[:, :2].max(axis=0)
            plane.pose.position.x = float((mins[0] + maxs[0]) / 2.0)
            plane.pose.position.y = float((mins[1] + maxs[1]) / 2.0)
            plane.pose.position.z = float(debug.reference_z_m)
            plane.pose.orientation.w = 1.0
            plane.scale.x = max(float(maxs[0] - mins[0]), 0.001)
            plane.scale.y = max(float(maxs[1] - mins[1]), 0.001)
            plane.scale.z = 0.004
            plane.color.r, plane.color.g, plane.color.b, plane.color.a = 0.1, 1.0, 0.2, 0.28
            markers.markers.append(plane)

        for index, measurement in enumerate(measurements):
            box = Marker()
            box.header = source_msg.header
            box.ns = "measured_boxes"
            box.id = index
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(measurement.center_xyz_m[0])
            box.pose.position.y = float(measurement.center_xyz_m[1])
            box.pose.position.z = float(measurement.center_xyz_m[2])
            box.pose.orientation.z = float(np.sin(measurement.yaw_rad / 2.0))
            box.pose.orientation.w = float(np.cos(measurement.yaw_rad / 2.0))
            box.scale.x = max(float(measurement.size_xyz_m[0]), 0.001)
            box.scale.y = max(float(measurement.size_xyz_m[1]), 0.001)
            box.scale.z = max(float(measurement.size_xyz_m[2]), 0.001)
            box.color.r, box.color.g, box.color.b, box.color.a = 1.0, 0.65, 0.0, 0.35
            markers.markers.append(box)

            center = Marker()
            center.header = source_msg.header
            center.ns = "box_centers"
            center.id = index
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = box.pose.position.x
            center.pose.position.y = box.pose.position.y
            center.pose.position.z = box.pose.position.z
            center.pose.orientation.w = 1.0
            center.scale.x = center.scale.y = center.scale.z = 0.025
            center.color.r, center.color.g, center.color.b, center.color.a = 1.0, 0.05, 0.05, 1.0
            markers.markers.append(center)

            text = Marker()
            text.header = source_msg.header
            text.ns = "box_dimensions"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = box.pose.position.x
            text.pose.position.y = box.pose.position.y
            text.pose.position.z = box.pose.position.z - box.scale.z / 2.0 - 0.04
            text.pose.orientation.w = 1.0
            text.scale.z = 0.035
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = (
                f"box {index}: center=({box.pose.position.x:.3f}, "
                f"{box.pose.position.y:.3f}, {box.pose.position.z:.3f}) m\n"
                f"size=({box.scale.x:.3f}, {box.scale.y:.3f}, {box.scale.z:.3f}) m "
                f"yaw={measurement.yaw_rad:.2f} rad"
            )
            markers.markers.append(text)
        return markers

    def _convert_depth_image(self, msg: Image) -> Optional[np.ndarray]:
        try:
            if self._bridge is not None:
                cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            else:
                # cv_bridge가 없으면 현재 지원하는 encoding만 수동 변환한다.
                dtype = np.uint16 if msg.encoding == "16UC1" else np.float32
                cv_image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"depth 이미지 변환 실패: {exc}")
            return None

        if msg.encoding == "16UC1":
            depth_m = cv_image.astype(np.float32) / 1000.0  # mm -> m
        else:
            depth_m = cv_image.astype(np.float32)  # 이미 미터 단위(32FC1)

        # invalid depth는 point cloud 생성 전에 제거한다.
        invalid_mask = ~np.isfinite(depth_m)
        invalid_count = int(np.count_nonzero(invalid_mask))
        if invalid_count > 0:
            depth_m = depth_m.copy()
            depth_m[invalid_mask] = 0.0
            self.get_logger().info(
                f"depth invalid values filtered: {invalid_count}",
                throttle_duration_sec=2.0,
            )
        return depth_m

    def _apply_depth_roi(self, depth_m: np.ndarray) -> np.ndarray:
        # 컨베이어 영역만 남기고 나머지 depth는 0으로 마스킹한다.
        if not bool(self.get_parameter("use_depth_roi").value):
            return depth_m

        h, w = depth_m.shape
        x_min = int(self.get_parameter("roi_x_min_px").value)
        y_min = int(self.get_parameter("roi_y_min_px").value)
        x_max = int(self.get_parameter("roi_x_max_px").value)
        y_max = int(self.get_parameter("roi_y_max_px").value)

        x_min = max(0, min(w - 1, x_min))
        x_max = max(0, min(w - 1, x_max))
        y_min = max(0, min(h - 1, y_min))
        y_max = max(0, min(h - 1, y_max))

        if x_min > x_max or y_min > y_max:
            self.get_logger().warn("depth ROI bounds are invalid; skipping ROI crop")
            return depth_m

        roi_depth = np.zeros_like(depth_m)
        roi_depth[y_min : y_max + 1, x_min : x_max + 1] = depth_m[y_min : y_max + 1, x_min : x_max + 1]
        return roi_depth

    def _publish_detections(
        self, selected_measurement: Optional[BoxMeasurement], frame_id: str
    ) -> None:
        if selected_measurement is not None:
            selected_measurement.box_id = self._next_box_id
            self._next_box_id += 1

        payload = {
            "stamp": self.get_clock().now().to_msg().sec,
            "frame_id": frame_id,
            "box": selected_measurement.to_dict() if selected_measurement is not None else None,
        }
        out_msg = String()
        out_msg.data = json.dumps(payload)
        self._detections_pub.publish(out_msg)

        if selected_measurement is not None:
            self.get_logger().info(
                f"box published: id={selected_measurement.box_id} frame={frame_id}"
            )
        else:
            self.get_logger().info("no box detected")

    def _transform_measurements_to_output_frame(
        self,
        measurements: List[BoxMeasurement],
        source_frame: str,
        stamp,
    ) -> str:
        target_frame = str(self._output_frame)
        if not measurements or not source_frame or source_frame == target_frame:
            return source_frame or target_frame

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=1.0),
            )
        except tf2_ros.ExtrapolationException:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"failed to transform detections {source_frame} -> {target_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return source_frame

        rotation = quaternion_to_rotation_matrix(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )

        for measurement in measurements:
            center_src = measurement.center_xyz_m.astype(np.float64)
            center_dst = rotation @ center_src + translation

            # yaw는 카메라 XY 평면의 방향 벡터를 target frame으로 회전시켜 다시 계산한다.
            axis_src = np.array(
                [np.cos(measurement.yaw_rad), np.sin(measurement.yaw_rad), 0.0],
                dtype=np.float64,
            )
            axis_dst = rotation @ axis_src

            measurement.center_xyz_m = center_dst.astype(np.float32)
            measurement.yaw_rad = float(np.arctan2(axis_dst[1], axis_dst[0]))

        return target_frame

def main(args=None) -> None:
    ros_args = list(sys.argv if args is None else args)
    has_params_file = any(
        arg == "--params-file" or arg.startswith("--params-file=")
        for arg in ros_args
    )
    if not has_params_file:
        params_file = (
            Path(get_package_share_directory("conveyor_box_measurement"))
            / "config"
            / "measurement.yaml"
        )
        ros_args = with_default_params_file(ros_args, params_file)
    rclpy.init(args=ros_args)
    node = ConveyorBoxMeasurementNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
