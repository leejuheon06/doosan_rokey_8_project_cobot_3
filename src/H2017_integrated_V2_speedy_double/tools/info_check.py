# -*- coding: utf-8 -*-
"""팔레타이징 계획 토픽을 받아 JSON 파일로 저장하는 ROS2 노드.

시뮬레이터(test5_2robot.py)가 DeepPack3D로 미리 계산한 <목표 배치 정보>를
std_msgs/msg/String에 JSON으로 실어 발행하면, 이 노드가 그것을 받아 파일로
남긴다. 받은 계획을 눈으로 검산하거나 다른 프로그램에 넘길 때 쓴다.

기대하는 메시지 형식 (배열, 적재 순서대로):
  [
    {"id": 0, "width": 0.20, "length": 0.30, "height": 0.18,
     "x": 0.15, "y": -0.20, "bottom_z": 0.15, "yaw": 90.0},
    ...
  ]

  id                    박스 생성(=적재) 순서. 0부터.
  width/length/height   원본 박스 치수 (yaw 적용 전, m).
  x, y                  팔레트 중심 기준 상대 좌표 (m).
  bottom_z              팔레트 윗면부터 박스 바닥까지의 높이 (m).
  yaw                   0.0 또는 90.0 (deg).

Isaac Sim과 무관한 순수 rclpy 노드라 시스템 python3로 실행한다.
(Isaac Sim 파이썬은 3.11이라 Humble의 rclpy를 import 할 수 없다.)

주의: 이 노드를 띄우는 셸에서는 isaac_ros를 실행하면 안 된다. Isaac Sim이
번들한 ROS 라이브러리 경로가 LD_LIBRARY_PATH에 끼면 시스템 rclpy가
librcl_logging_spdlog.so undefined symbol로 깨진다.

실행 예:
  source /opt/ros/humble/setup.bash
  python3 tools/info_check.py
  python3 tools/info_check.py --output /tmp/plan.json
  python3 tools/info_check.py --topic /palletizing/plan
"""

import argparse
import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import String

# test5_2robot.py의 PLAN_TOPIC_NAME과 같아야 한다.
DEFAULT_TOPIC = "/palletizing/plan"
DEFAULT_OUTPUT_PATH = str(Path(__file__).resolve().parents[1] / "outputs" / "palletizing_plan.json")
NODE_NAME = "palletizing_plan_recorder"

# 계획 항목 하나가 반드시 담고 있어야 하는 값들.
REQUIRED_FIELDS = (
    "id",
    "width",
    "length",
    "height",
    "x",
    "y",
    "bottom_z",
    "yaw",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="팔레타이징 계획 토픽을 JSON 파일로 저장한다.",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"구독할 std_msgs/msg/String 토픽 (기본: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_PATH,
        help=f"저장할 JSON 파일 경로 (기본: {DEFAULT_OUTPUT_PATH})",
    )
    args, _unknown = parser.parse_known_args()
    return args


def plan_qos(durability) -> QoSProfile:
    """계획처럼 "마지막 한 개"만 의미 있는 데이터에 맞춘 QoS."""
    profile = QoSProfile(depth=1)
    profile.history = QoSHistoryPolicy.KEEP_LAST
    profile.reliability = QoSReliabilityPolicy.RELIABLE
    profile.durability = durability
    return profile


def describe_problems(plan) -> list[str]:
    """스키마에서 어긋난 점을 사람이 읽을 문장으로 모은다. 비어 있으면 정상.

    저장을 막지는 않는다. 형식이 조금 달라도 받은 내용을 남기는 편이 낫고,
    무엇이 이상한지는 로그로 알려 주면 충분하다.
    """
    if not isinstance(plan, list):
        return [f"최상위가 배열이 아니라 {type(plan).__name__} 입니다."]

    problems = []
    for index, entry in enumerate(plan):
        if not isinstance(entry, dict):
            problems.append(
                f"[{index}] 객체가 아니라 {type(entry).__name__} 입니다."
            )
            continue
        missing = [name for name in REQUIRED_FIELDS if name not in entry]
        if missing:
            problems.append(f"[{index}] 누락된 항목: {', '.join(missing)}")
        for name in REQUIRED_FIELDS:
            if name not in entry:
                continue
            value = entry[name]
            # bool은 int의 하위 타입이라 따로 걸러야 숫자로 오인하지 않는다.
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                problems.append(
                    f"[{index}] {name}이 숫자가 아닙니다: {value!r}"
                )
    return problems


class PlanRecorder(Node):
    """계획 토픽을 구독해 마지막 계획을 JSON 파일로 유지한다."""

    def __init__(self, topic: str, output_path: str) -> None:
        super().__init__(NODE_NAME)
        self.output_path = Path(output_path).expanduser()
        if self.output_path.parent != Path(""):
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.last_payload = None

        # 발행 측 QoS를 모르므로 두 durability로 모두 구독한다.
        # transient_local 발행이면 이 노드를 늦게 켜도 마지막 계획을 받고,
        # volatile 발행이면 transient_local 구독만 걸었을 때 생기는 QoS 불일치
        # (=아무것도 못 받는 상태)를 피한다. 같은 메시지가 두 경로로 들어오면
        # 두 번째는 on_plan에서 무시한다.
        for durability in (
            QoSDurabilityPolicy.TRANSIENT_LOCAL,
            QoSDurabilityPolicy.VOLATILE,
        ):
            self.create_subscription(
                String, topic, self.on_plan, plan_qos(durability)
            )

        self.get_logger().info(
            f"'{topic}' 구독 시작 — 저장 위치: {self.output_path}"
        )

    def on_plan(self, msg: String) -> None:
        if msg.data == self.last_payload:
            # 두 구독으로 같은 계획이 두 번 들어온 경우.
            return
        self.last_payload = msg.data

        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(
                f"JSON 파싱 실패 — 저장하지 않습니다: {exc}"
            )
            return

        for problem in describe_problems(plan):
            self.get_logger().warn(f"형식 확인: {problem}")

        self.output_path.write_text(
            json.dumps(plan, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        count = len(plan) if isinstance(plan, list) else 1
        self.get_logger().info(
            f"저장 완료: {self.output_path} (박스 {count}개)"
        )


def main():
    args = parse_args()
    rclpy.init()
    node = PlanRecorder(args.topic, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl-C. 정상 종료이므로 조용히 빠져나간다.
        print()
    except ExternalShutdownException:
        # SIGTERM 등으로 rclpy 컨텍스트가 먼저 내려간 경우. 역시 정상 종료다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
