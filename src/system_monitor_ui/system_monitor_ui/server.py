"""Flask dashboard bridged to the Speedy Double launcher through ROS 2."""

from __future__ import annotations

import atexit
from io import BytesIO
import json
import math
import struct
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, redirect, render_template, request, url_for
from PIL import Image as PilImage

from . import db
from .launcher import CONTROL_TOPIC, LOG_TOPIC, PROCESS_STATE_TOPIC, RESULT_TOPIC


VISION_DETECTION_TOPICS = (
    "/vision/conveyor_1/box_detections",
    "/vision/conveyor_2/box_detections",
)
VISION_OVERLAY_TOPICS = (
    "/vision/conveyor_1/debug/overlay_image",
    "/vision/conveyor_2/debug/overlay_image",
)
VISION_POINTCLOUD_TOPICS = (
    "/vision/conveyor_1/debug/pointcloud",
    "/vision/conveyor_2/debug/pointcloud",
)
VISION_RAISED_POINT_TOPICS = (
    "/vision/conveyor_1/debug/raised_points",
    "/vision/conveyor_2/debug/raised_points",
)
VISION_MARKER_TOPICS = (
    "/vision/conveyor_1/debug/markers",
    "/vision/conveyor_2/debug/markers",
)


def _ros_image_to_jpeg(msg, *, quality: int = 82) -> bytes:
    """Encode the image formats used by the vision debug overlay as JPEG."""
    raw_modes = {
        "bgr8": ("RGB", "BGR", 3),
        "rgb8": ("RGB", "RGB", 3),
        "bgra8": ("RGB", "BGRA", 4),
        "rgba8": ("RGB", "RGBA", 4),
        "mono8": ("L", "L", 1),
    }
    try:
        mode, raw_mode, channels = raw_modes[msg.encoding.lower()]
    except KeyError as exc:
        raise ValueError(f"unsupported vision image encoding: {msg.encoding}") from exc
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0 or step < width * channels:
        raise ValueError("invalid vision image dimensions")
    required = step * height
    data = bytes(msg.data)
    if len(data) < required:
        raise ValueError(f"short vision image: {len(data)} < {required}")
    image = PilImage.frombytes(mode, (width, height), data[:required], "raw", raw_mode, step, 1)
    output = BytesIO()
    image.save(output, format="JPEG", quality=quality, optimize=False)
    return output.getvalue()


def _ros_pointcloud_to_xyz(msg, *, max_points: int = 5000) -> tuple[bytes, int]:
    """Return a browser-friendly little-endian packed XYZ point sample."""
    if max_points < 1:
        raise ValueError("max_points must be positive")
    offsets = {field.name: int(field.offset) for field in msg.fields}
    if not all(axis in offsets for axis in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain x, y and z fields")
    point_step = int(msg.point_step)
    total = int(msg.width) * int(msg.height)
    raw = memoryview(msg.data)
    if point_step <= 0 or len(raw) < total * point_step:
        raise ValueError("invalid PointCloud2 storage")
    sample_step = max(1, math.ceil(total / max_points))
    endian = ">" if bool(msg.is_bigendian) else "<"
    unpack = struct.Struct(endian + "f").unpack_from
    packed = bytearray()
    append = packed.extend
    count = 0
    for point_index in range(0, total, sample_step):
        base = point_index * point_step
        xyz = tuple(unpack(raw, base + offsets[axis])[0] for axis in ("x", "y", "z"))
        if all(math.isfinite(value) for value in xyz):
            append(struct.pack("<fff", *xyz))
            count += 1
    return bytes(packed), count


def _resolve_templates_dir() -> Path:
    source = Path(__file__).resolve().parent.parent / "templates"
    if source.is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("system_monitor_ui")) / "templates"


class MonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.launcher_online = False
        self.last_heartbeat = 0.0
        self.process = {
            "status": "idle", "pid": None, "run_uid": None,
            "elapsed_seconds": 0.0, "settings": {}, "error": None,
        }
        self.logs: list[dict] = []
        self.result: dict | None = None
        self.saved_run_uids: set[str] = set()
        self.vision = [
            {
                "frame_received_at": 0.0,
                "detection_received_at": 0.0,
                "frame_sequence": 0,
                "cloud_received_at": 0.0,
                "cloud_sequence": 0,
                "cloud_count": 0,
                "raised_sequence": 0,
                "raised_count": 0,
                "marker_received_at": 0.0,
                "marker_sequence": 0,
                "reference_plane": None,
                "measured_boxes": [],
                "box": None,
                "frame_id": None,
                "error": None,
            }
            for _ in range(2)
        ]
        self.vision_frames: list[bytes | None] = [None, None]
        self.vision_clouds = [
            {"roi": None, "raised": None}
            for _ in range(2)
        ]

    def snapshot(self, ros_available: bool) -> dict:
        with self.lock:
            online = self.launcher_online and time.monotonic() - self.last_heartbeat < 3.5
            now = time.monotonic()
            vision = []
            for index, item in enumerate(self.vision):
                frame_age = now - item["frame_received_at"] if item["frame_received_at"] else None
                detection_age = (
                    now - item["detection_received_at"]
                    if item["detection_received_at"] else None
                )
                cloud_age = now - item["cloud_received_at"] if item["cloud_received_at"] else None
                vision.append({
                    "line": index + 1,
                    "connected": frame_age is not None and frame_age < 2.5,
                    "frame_age_seconds": frame_age,
                    "detection_age_seconds": detection_age,
                    "frame_sequence": item["frame_sequence"],
                    "cloud_connected": cloud_age is not None and cloud_age < 2.5,
                    "cloud_age_seconds": cloud_age,
                    "cloud_sequence": item["cloud_sequence"],
                    "cloud_count": item["cloud_count"],
                    "raised_sequence": item["raised_sequence"],
                    "raised_count": item["raised_count"],
                    "marker_sequence": item["marker_sequence"],
                    "reference_plane": (
                        dict(item["reference_plane"])
                        if isinstance(item["reference_plane"], dict) else None
                    ),
                    "measured_boxes": [dict(box) for box in item["measured_boxes"]],
                    "box": dict(item["box"]) if isinstance(item["box"], dict) else None,
                    "frame_id": item["frame_id"],
                    "error": item["error"],
                })
            return {
                "ros_available": ros_available,
                "launcher_online": online,
                "process": dict(self.process),
                "logs": list(self.logs[-300:]),
                "result": dict(self.result) if self.result else None,
                "vision": vision,
            }

    def vision_frame(self, line_index: int) -> bytes | None:
        with self.lock:
            return self.vision_frames[line_index]

    def vision_cloud(self, line_index: int, kind: str) -> bytes | None:
        with self.lock:
            return self.vision_clouds[line_index][kind]


class RosBridge:
    def __init__(self, state: MonitorState):
        self.state = state
        self.active = False
        self.node = None
        self.rclpy = None
        self.publisher = None
        try:
            import rclpy
            from rclpy.executors import ExternalShutdownException
            from rclpy.node import Node
            from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
            from sensor_msgs.msg import Image, PointCloud2
            from std_msgs.msg import String
            from visualization_msgs.msg import MarkerArray

            if not rclpy.ok():
                rclpy.init(args=None)
            node = Node("system_monitor_ui_bridge")
            latched = QoSProfile(depth=1)
            latched.reliability = QoSReliabilityPolicy.RELIABLE
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = node.create_publisher(String, CONTROL_TOPIC, 10)
            node.create_subscription(String, PROCESS_STATE_TOPIC, self._on_state, latched)
            node.create_subscription(String, RESULT_TOPIC, self._on_result, latched)
            node.create_subscription(String, LOG_TOPIC, self._on_log, 100)
            sensor_qos = QoSProfile(depth=2)
            sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            for line_index, topic in enumerate(VISION_DETECTION_TOPICS):
                node.create_subscription(
                    String,
                    topic,
                    lambda msg, index=line_index: self._on_vision_detection(index, msg),
                    10,
                )
            for line_index, topic in enumerate(VISION_OVERLAY_TOPICS):
                node.create_subscription(
                    Image,
                    topic,
                    lambda msg, index=line_index: self._on_vision_frame(index, msg),
                    sensor_qos,
                )
            for line_index, topic in enumerate(VISION_POINTCLOUD_TOPICS):
                node.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, index=line_index: self._on_vision_cloud(index, "roi", msg),
                    sensor_qos,
                )
            for line_index, topic in enumerate(VISION_RAISED_POINT_TOPICS):
                node.create_subscription(
                    PointCloud2,
                    topic,
                    lambda msg, index=line_index: self._on_vision_cloud(index, "raised", msg),
                    sensor_qos,
                )
            for line_index, topic in enumerate(VISION_MARKER_TOPICS):
                node.create_subscription(
                    MarkerArray,
                    topic,
                    lambda msg, index=line_index: self._on_vision_markers(index, msg),
                    10,
                )
            self.rclpy = rclpy
            self.node = node
            self.String = String
            self.active = True
            def spin_node():
                try:
                    rclpy.spin(node)
                except ExternalShutdownException:
                    pass

            threading.Thread(target=spin_node, daemon=True).start()
        except Exception as exc:
            with state.lock:
                state.process["error"] = f"ROS bridge unavailable: {exc}"

    def _decode(self, msg) -> dict:
        value = json.loads(msg.data)
        if not isinstance(value, dict):
            raise ValueError("ROS JSON payload must be an object")
        return value

    def _on_state(self, msg) -> None:
        try:
            payload = self._decode(msg)
            with self.state.lock:
                self.state.launcher_online = bool(payload.get("launcher_online"))
                self.state.last_heartbeat = time.monotonic()
                self.state.process.update(payload)
        except Exception as exc:
            print(f"[system_monitor_ui] invalid process state: {exc}")

    def _on_log(self, msg) -> None:
        try:
            payload = self._decode(msg)
            with self.state.lock:
                if payload.get("run_uid") != self.state.process.get("run_uid"):
                    self.state.logs = []
                self.state.logs.append(payload)
                del self.state.logs[:-500]
        except Exception as exc:
            print(f"[system_monitor_ui] invalid log: {exc}")

    def _on_result(self, msg) -> None:
        try:
            payload = self._decode(msg)
            run_uid = str(payload.get("run_uid") or "")
            with self.state.lock:
                self.state.result = payload
                already_saved = run_uid in self.state.saved_run_uids
                if run_uid:
                    self.state.saved_run_uids.add(run_uid)
            if run_uid and not already_saved:
                db.save_execution(payload)
        except Exception as exc:
            print(f"[system_monitor_ui] invalid result: {exc}")

    def _on_vision_detection(self, line_index: int, msg) -> None:
        try:
            payload = self._decode(msg)
            box = payload.get("box")
            if box is not None and not isinstance(box, dict):
                raise ValueError("vision box must be an object or null")
            with self.state.lock:
                item = self.state.vision[line_index]
                item["box"] = box
                item["frame_id"] = payload.get("frame_id")
                item["detection_received_at"] = time.monotonic()
                item["error"] = None
        except Exception as exc:
            with self.state.lock:
                self.state.vision[line_index]["error"] = str(exc)
            print(f"[system_monitor_ui] invalid C{line_index + 1} detection: {exc}")

    def _on_vision_frame(self, line_index: int, msg) -> None:
        try:
            frame = _ros_image_to_jpeg(msg)
            with self.state.lock:
                item = self.state.vision[line_index]
                self.state.vision_frames[line_index] = frame
                item["frame_received_at"] = time.monotonic()
                item["frame_sequence"] += 1
                item["error"] = None
        except Exception as exc:
            with self.state.lock:
                self.state.vision[line_index]["error"] = str(exc)
            print(f"[system_monitor_ui] invalid C{line_index + 1} overlay: {exc}")

    def _on_vision_cloud(self, line_index: int, kind: str, msg) -> None:
        try:
            limit = 5000 if kind == "roi" else 3500
            cloud, count = _ros_pointcloud_to_xyz(msg, max_points=limit)
            with self.state.lock:
                item = self.state.vision[line_index]
                self.state.vision_clouds[line_index][kind] = cloud
                item[f"{kind if kind == 'raised' else 'cloud'}_sequence"] += 1
                item[f"{kind if kind == 'raised' else 'cloud'}_count"] = count
                item["cloud_received_at"] = time.monotonic()
                item["error"] = None
        except Exception as exc:
            with self.state.lock:
                self.state.vision[line_index]["error"] = str(exc)
            print(f"[system_monitor_ui] invalid C{line_index + 1} {kind} cloud: {exc}")

    def _on_vision_markers(self, line_index: int, msg) -> None:
        try:
            plane = None
            boxes = []
            for marker in msg.markers:
                if marker.ns == "reference_plane" and marker.action == marker.ADD:
                    plane = {
                        "center_m": [
                            float(marker.pose.position.x),
                            float(marker.pose.position.y),
                            float(marker.pose.position.z),
                        ],
                        "size_m": [
                            float(marker.scale.x),
                            float(marker.scale.y),
                            float(marker.scale.z),
                        ],
                    }
                elif marker.ns == "measured_boxes" and marker.action == marker.ADD:
                    boxes.append({
                        "center_m": [
                            float(marker.pose.position.x),
                            float(marker.pose.position.y),
                            float(marker.pose.position.z),
                        ],
                        "size_m": [
                            float(marker.scale.x),
                            float(marker.scale.y),
                            float(marker.scale.z),
                        ],
                        "yaw_rad": float(
                            2.0 * math.atan2(
                                marker.pose.orientation.z,
                                marker.pose.orientation.w,
                            )
                        ),
                    })
            with self.state.lock:
                item = self.state.vision[line_index]
                if plane is not None:
                    item["reference_plane"] = plane
                item["measured_boxes"] = boxes
                item["marker_received_at"] = time.monotonic()
                item["marker_sequence"] += 1
        except Exception as exc:
            with self.state.lock:
                self.state.vision[line_index]["error"] = str(exc)
            print(f"[system_monitor_ui] invalid C{line_index + 1} markers: {exc}")

    def publish_control(self, payload: dict) -> None:
        if not self.active or self.publisher is None:
            raise RuntimeError("ROS 2 bridge is not available")
        msg = self.String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)

    def shutdown(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()


def create_app() -> Flask:
    app = Flask(__name__, template_folder=str(_resolve_templates_dir()))
    state = MonitorState()
    bridge = RosBridge(state)
    db.init_db()
    atexit.register(bridge.shutdown)
    app.config.update(STATE=state, ROS_BRIDGE=bridge)

    @app.route("/")
    def index():
        return redirect(url_for("smu_page"))

    @app.route("/smu")
    def smu_page():
        return render_template("smu.html")

    @app.route("/db")
    def db_page():
        return render_template("db.html")

    @app.route("/api/state")
    def api_state():
        return jsonify(state.snapshot(bridge.active))

    @app.route("/api/vision/<int:line>/frame.jpg")
    def api_vision_frame(line: int):
        if line not in (1, 2):
            return jsonify({"error": "line must be 1 or 2"}), 404
        frame = state.vision_frame(line - 1)
        if frame is None:
            return Response(status=204)
        return Response(
            frame,
            mimetype="image/jpeg",
            headers={"Cache-Control": "no-store, max-age=0"},
        )

    @app.route("/api/vision/<int:line>/cloud.bin")
    def api_vision_cloud(line: int):
        if line not in (1, 2):
            return jsonify({"error": "line must be 1 or 2"}), 404
        kind = request.args.get("kind", "roi")
        if kind not in ("roi", "raised"):
            return jsonify({"error": "kind must be roi or raised"}), 400
        cloud = state.vision_cloud(line - 1, kind)
        if cloud is None:
            return Response(status=204)
        return Response(
            cloud,
            mimetype="application/octet-stream",
            headers={
                "Cache-Control": "no-store, max-age=0",
                "X-Point-Count": str(len(cloud) // 12),
            },
        )

    @app.route("/api/control", methods=["POST"])
    def api_control():
        payload = request.get_json(silent=True) or {}
        action = payload.get("action")
        if action not in ("start", "stop"):
            return jsonify({"error": "action must be start or stop"}), 400
        if action == "start" and not isinstance(payload.get("settings"), dict):
            return jsonify({"error": "settings object is required"}), 400
        try:
            bridge.publish_control(payload)
        except RuntimeError as exc:
            return jsonify({"error": str(exc)}), 503
        if action == "start":
            with state.lock:
                state.logs = []
                state.result = None
        return jsonify({"ok": True})

    @app.route("/api/runs")
    def api_runs():
        return jsonify(db.fetch_executions(limit=min(request.args.get("limit", 100, type=int), 500)))

    @app.route("/api/runs/<int:run_id>")
    def api_run(run_id: int):
        item = db.fetch_execution(run_id)
        return jsonify(item) if item else (jsonify({"error": "not found"}), 404)

    @app.route("/api/runs/<int:run_id>", methods=["DELETE"])
    def api_delete_run(run_id: int):
        return jsonify({"deleted": db.delete_execution(run_id)})

    @app.route("/api/runs", methods=["DELETE"])
    def api_clear_runs():
        return jsonify({"deleted": db.clear_executions()})

    return app


def main() -> None:
    create_app().run(host="0.0.0.0", port=5000, debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
