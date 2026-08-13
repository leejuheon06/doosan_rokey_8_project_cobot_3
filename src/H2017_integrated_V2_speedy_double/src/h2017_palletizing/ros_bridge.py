
"""The single ROS2 node used by the palletizing simulation."""

from __future__ import annotations

import json
import sys
from pathlib import Path

from .config import (
    CONVEYOR_TRIGGER_TOPICS,
    PLAN_NODE_NAME,
    PLAN_TOPIC_NAME,
    VISION_DETECTION_TOPICS,
)


class ConveyorRosChannel:
    """IntakeStation에 한 라인의 ROS 입출력만 노출한다."""

    def __init__(self, bridge: "PalletizingRosBridge", index: int) -> None:
        self.bridge = bridge
        self.index = index

    def publish_conveyor_stopped(self) -> None:
        self.bridge.publish_conveyor_stopped(self.index)

    def poll_detection(self) -> str | None:
        return self.bridge.poll_detection(self.index)


class PalletizingRosBridge:
    """Publish the fixed packing plan and conveyor stop state."""

    def __init__(self) -> None:
        self.node = None
        self.plan_publisher = None
        self.conveyor_publishers = []
        self._string_type = None
        self._bool_type = None
        self._rclpy = None
        self._latest_detections = [None] * len(VISION_DETECTION_TOPICS)
        self._subscriptions = []
        self._channels = [
            ConveyorRosChannel(self, index)
            for index in range(len(CONVEYOR_TRIGGER_TOPICS))
        ]
        try:
            # SimulationApp may rebuild sys.path during startup, so restore the
            # bundled Humble Python packages immediately before importing rclpy.
            isaac_root = Path(sys.executable).parents[3]
            ros_python_dir = isaac_root / "exts/isaacsim.ros2.bridge/humble/rclpy"
            if str(ros_python_dir) not in sys.path:
                sys.path.insert(0, str(ros_python_dir))
            import rclpy
            from rclpy.qos import (
                QoSDurabilityPolicy,
                QoSHistoryPolicy,
                QoSProfile,
                QoSReliabilityPolicy,
            )
            from std_msgs.msg import Bool, String

            if not rclpy.ok():
                rclpy.init()
            qos = QoSProfile(depth=1)
            qos.history = QoSHistoryPolicy.KEEP_LAST
            qos.reliability = QoSReliabilityPolicy.RELIABLE
            qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

            self.node = rclpy.create_node(PLAN_NODE_NAME)
            self.plan_publisher = self.node.create_publisher(
                String, PLAN_TOPIC_NAME, qos
            )
            self.conveyor_publishers = [
                self.node.create_publisher(Bool, topic, qos)
                for topic in CONVEYOR_TRIGGER_TOPICS
            ]
            self._rclpy = rclpy
            self._string_type = String
            self._bool_type = Bool
            print(f"[ROS2] 단일 브리지 노드 준비 완료: {PLAN_NODE_NAME}")
        except Exception as exc:
            print(
                f"[ROS2 경고] 브리지 준비 실패 — 시뮬레이션은 계속합니다: "
                f"{type(exc).__name__}: {exc}",
                flush=True,
            )

    def publish_plan(self, payload: list[dict]) -> None:
        if self.plan_publisher is None:
            print("[PLAN 발행 생략] ROS2가 준비되지 않았습니다.", flush=True)
            return
        message = self._string_type()
        message.data = json.dumps(payload, indent=2, ensure_ascii=False)
        self.plan_publisher.publish(message)
        print(
            f"[PLAN 발행] {PLAN_TOPIC_NAME} <- 박스 {len(payload)}개",
            flush=True,
        )

    def line(self, conveyor_id: int) -> ConveyorRosChannel:
        if conveyor_id not in (1, 2):
            raise ValueError(f"conveyor_id는 1 또는 2여야 합니다: {conveyor_id}")
        return self._channels[conveyor_id - 1]

    def publish_conveyor_stopped(self, index: int) -> None:
        if not self.conveyor_publishers:
            return
        message = self._bool_type()
        message.data = True
        self.conveyor_publishers[index].publish(message)

    def subscribe_detections(self) -> None:
        """두 비전 노드의 결과를 라인별 큐에 독립적으로 보관한다.

        각 IntakeStation은 자기 채널만 꺼내므로 두 카메라가 같은 physics step에
        응답해도 다른 라인의 결과를 소비하지 않는다.
        """
        if self.node is None:
            print("[검출 구독 생략] ROS2가 준비되지 않았습니다.", flush=True)
            return
        for index, topic in enumerate(VISION_DETECTION_TOPICS):
            def on_detection(message, line_index=index):
                self._latest_detections[line_index] = message.data

            subscription = self.node.create_subscription(
                self._string_type, topic, on_detection, 10
            )
            self._subscriptions.append(subscription)
            print(f"[ROS2] 검출 구독 C{index + 1}: {topic}", flush=True)

    def poll_detection(self, index: int) -> str | None:
        """콜백을 한 번 돌리고, 새로 받은 payload가 있으면 꺼내 준다.

        같은 측정을 두 번 세지 않도록 꺼낼 때 비운다.
        """
        if self.node is None:
            return None
        self._rclpy.spin_once(self.node, timeout_sec=0.0)
        payload = self._latest_detections[index]
        self._latest_detections[index] = None
        return payload

    def close(self) -> None:
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
