"""ROS 2 launcher for the Speedy Double Isaac Sim process.

The web server never executes shell text.  It publishes a validated JSON
request on /palletizing/control; this node converts the allow-listed settings
to argv, starts Isaac Sim, and publishes state/log/result messages.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import queue
import re
import signal
import shutil
import subprocess
import threading
import time
import uuid


CONTROL_TOPIC = "/palletizing/control"
PROCESS_STATE_TOPIC = "/palletizing/process_state"
LOG_TOPIC = "/palletizing/log"
RESULT_TOPIC = "/palletizing/result"

PROJECT_DIR_NAME = "H2017_integrated_V2_speedy_double"


def resolve_project_root() -> Path:
    """Find the simulation project without depending on a user-specific path."""
    configured = os.environ.get("SPEEDY_DOUBLE_PROJECT")
    if configured:
        candidates = [Path(configured).expanduser()]
    else:
        candidates = []
        for anchor in (Path.cwd(), Path(__file__).resolve()):
            candidates.extend((anchor, *anchor.parents))

    for candidate in candidates:
        project = candidate if candidate.name == PROJECT_DIR_NAME else candidate / PROJECT_DIR_NAME
        if (project / "scripts" / "test5_2robot.py").is_file():
            return project.resolve()
    raise FileNotFoundError(
        f"Cannot find {PROJECT_DIR_NAME}. Run the launcher from the repository root "
        "or set SPEEDY_DOUBLE_PROJECT."
    )


def resolve_isaac_python() -> Path:
    """Resolve Isaac Sim's Python launcher from portable environment settings."""
    configured = os.environ.get("ISAAC_SIM_PYTHON") or os.environ.get("ISAAC_PY")
    if configured:
        candidate = Path(configured).expanduser()
        if candidate.is_file():
            return candidate.resolve()
        raise FileNotFoundError(f"Isaac Sim Python launcher not found: {candidate}")

    executable = shutil.which("isaacsim-python")
    if executable:
        return Path(executable).resolve()
    raise FileNotFoundError(
        "Set ISAAC_SIM_PYTHON to the Isaac Sim python.sh path before starting the launcher."
    )

PACKING_METHODS = {"bl", "baf", "bssf", "blsf"}
PLACERS = {"default", "stable"}
STABILITY_POLICIES = {"strict", "continue-drift"}
BOX_NUMBER_PATTERN = re.compile(r"^[1-4](?:-[1-4]|(?:,[1-4])*)?$")


def _integer(value, name: str, low: int, high: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if not low <= parsed <= high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return parsed


def _number(value, name: str, low: float, high: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not low <= parsed <= high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return parsed


def validate_settings(raw: dict) -> dict:
    """Return a normalized, allow-listed Speedy Double configuration."""
    if not isinstance(raw, dict):
        raise ValueError("settings must be an object")

    box_numbers = str(raw.get("box_numbers", "1-4")).strip()
    if not BOX_NUMBER_PATTERN.fullmatch(box_numbers):
        raise ValueError("box_numbers must look like 1-4 or 1,3,4")
    if "-" in box_numbers:
        start, end = map(int, box_numbers.split("-"))
        if start > end:
            raise ValueError("box_numbers range must be ascending")

    method = str(raw.get("packing_method", "baf"))
    placer = str(raw.get("placer", "default"))
    policy = str(raw.get("stability_policy", "strict"))
    if method not in PACKING_METHODS:
        raise ValueError("unknown packing_method")
    if placer not in PLACERS:
        raise ValueError("unknown placer")
    if policy not in STABILITY_POLICIES:
        raise ValueError("unknown stability_policy")

    seed_value = raw.get("seed")
    seed = None if seed_value in (None, "") else _integer(seed_value, "seed", -2**31, 2**31 - 1)
    return {
        "box_count": _integer(raw.get("box_count", 25), "box_count", 1, 500),
        "box_numbers": box_numbers,
        "seed": seed,
        "spawn_interval": _number(raw.get("spawn_interval", 5.0), "spawn_interval", 0.1, 3600.0),
        "measurement_settle_steps": _integer(
            raw.get("measurement_settle_steps", 0),
            "measurement_settle_steps", 0, 10000,
        ),
        "packing_method": method,
        "placer": placer,
        "stability_policy": policy,
        "stable_min_horizontal_support": _number(
            raw.get("stable_min_horizontal_support", 0.3),
            "stable_min_horizontal_support", 0.0, 1.0,
        ),
        "stable_min_com_margin": _number(
            raw.get("stable_min_com_margin", 0.05),
            "stable_min_com_margin", -1.0, 1.0,
        ),
        "freeze_after_stability": bool(raw.get("freeze_after_stability", True)),
        "yaw_rotation": bool(raw.get("yaw_rotation", False)),
        "stable_tiebreak": bool(raw.get("stable_tiebreak", False)),
        "center_stack_candidates": bool(raw.get("center_stack_candidates", False)),
        "headless": bool(raw.get("headless", False)),
        "ros_domain_id": _integer(raw.get("ros_domain_id", 129), "ros_domain_id", 0, 232),
    }


def build_command(
    settings: dict,
    summary_path: Path,
    *,
    project_root: Path | None = None,
    isaac_py: Path | None = None,
) -> list[str]:
    """Build argv without a shell so UI values cannot become commands."""
    project_root = project_root or resolve_project_root()
    isaac_py = isaac_py or resolve_isaac_python()
    script = Path(project_root) / "scripts" / "test5_2robot.py"
    argv = [
        str(isaac_py), str(script),
        "--box-count", str(settings["box_count"]),
        "--box-numbers", settings["box_numbers"],
        "--spawn-interval", str(settings["spawn_interval"]),
        "--measurement-settle-steps", str(settings["measurement_settle_steps"]),
        "--packing-method", settings["packing_method"],
        "--placer", settings["placer"],
        "--stability-policy", settings["stability_policy"],
        "--stable-min-horizontal-support", str(settings["stable_min_horizontal_support"]),
        "--stable-min-com-margin", str(settings["stable_min_com_margin"]),
        "--run-summary-path", str(summary_path),
        "--exit-on-complete",
    ]
    if settings["seed"] is not None:
        argv.extend(("--seed", str(settings["seed"])))
    argv.append(
        "--freeze-after-stability"
        if settings["freeze_after_stability"]
        else "--no-freeze-after-stability"
    )
    for enabled, option in (
        (settings["yaw_rotation"], "--yaw-rotation"),
        (settings["stable_tiebreak"], "--stable-tiebreak"),
        (settings["center_stack_candidates"], "--center-stack-candidates"),
        (settings["headless"], "--headless"),
    ):
        if enabled:
            argv.append(option)
    return argv


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    from std_msgs.msg import String

    class SpeedyDoubleLauncher(Node):
        def __init__(self):
            super().__init__("speedy_double_launcher")
            latched = QoSProfile(depth=1)
            latched.reliability = QoSReliabilityPolicy.RELIABLE
            latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.state_pub = self.create_publisher(String, PROCESS_STATE_TOPIC, latched)
            self.result_pub = self.create_publisher(String, RESULT_TOPIC, latched)
            self.log_pub = self.create_publisher(String, LOG_TOPIC, 100)
            self.create_subscription(String, CONTROL_TOPIC, self._on_control, 10)
            self.process: subprocess.Popen | None = None
            self.run_uid: str | None = None
            self.settings: dict = {}
            self.summary_path: Path | None = None
            self.started_at: float | None = None
            self.status = "idle"
            self.error: str | None = None
            self.lines: queue.Queue[str] = queue.Queue()
            self.log_tail: list[str] = []
            self.create_timer(0.1, self._poll)
            self.create_timer(1.0, self._publish_state)
            self._publish_state()
            self.get_logger().info("speedy_double_launcher started")

        def _message(self, payload: dict):
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            return msg

        def _publish_state(self) -> None:
            self.state_pub.publish(self._message({
                "launcher_online": True,
                "status": self.status,
                "run_uid": self.run_uid,
                "pid": self.process.pid if self.process and self.process.poll() is None else None,
                "settings": self.settings,
                "error": self.error,
                "elapsed_seconds": time.monotonic() - self.started_at if self.started_at else 0.0,
            }))

        def _publish_log(self, line: str) -> None:
            self.log_tail.append(line)
            del self.log_tail[:-200]
            self.log_pub.publish(self._message({
                "run_uid": self.run_uid, "line": line,
            }))

        def _on_control(self, msg) -> None:
            try:
                payload = json.loads(msg.data)
                action = payload.get("action")
                if action == "start":
                    self._start(payload.get("settings", {}))
                elif action == "stop":
                    self._stop()
                else:
                    raise ValueError("action must be start or stop")
            except Exception as exc:
                self.error = str(exc)
                self._publish_state()

        def _start(self, raw_settings: dict) -> None:
            if self.process is not None and self.process.poll() is None:
                raise RuntimeError("Speedy Double is already running")
            settings = validate_settings(raw_settings)
            project_root = resolve_project_root()
            isaac_py = resolve_isaac_python()

            self.run_uid = uuid.uuid4().hex
            self.summary_path = Path(f"/tmp/speedy_double_summary_{self.run_uid}.json")
            argv = build_command(
                settings,
                self.summary_path,
                project_root=project_root,
                isaac_py=isaac_py,
            )
            env = os.environ.copy()
            env.update({
                "ROS_DOMAIN_ID": str(settings["ros_domain_id"]),
                "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "PYTHONUNBUFFERED": "1",
            })
            self.settings = settings
            self.error = None
            self.status = "starting"
            self.started_at = time.monotonic()
            self.log_tail = []
            self.process = subprocess.Popen(
                argv,
                cwd=str(project_root),
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
            )
            threading.Thread(target=self._read_output, daemon=True).start()
            self.status = "running"
            self._publish_log("[UI] " + " ".join(argv))
            self._publish_state()

        def _read_output(self) -> None:
            assert self.process is not None and self.process.stdout is not None
            for line in self.process.stdout:
                self.lines.put(line.rstrip("\n"))

        def _stop(self) -> None:
            if self.process is None or self.process.poll() is not None:
                return
            self.status = "stopping"
            os.killpg(self.process.pid, signal.SIGINT)
            self._publish_state()

        def _poll(self) -> None:
            while True:
                try:
                    self._publish_log(self.lines.get_nowait())
                except queue.Empty:
                    break
            if self.process is None:
                return
            exit_code = self.process.poll()
            if exit_code is None:
                return

            result = None
            if self.summary_path and self.summary_path.is_file():
                try:
                    result = json.loads(self.summary_path.read_text(encoding="utf-8"))
                except Exception as exc:
                    self.error = f"summary read failed: {exc}"
            if result is None:
                result = {
                    "status": "stopped" if self.status == "stopping" else "failed",
                    "error": self.error or (self.log_tail[-1] if self.log_tail else "no summary produced"),
                }
            result.update({
                "run_uid": self.run_uid,
                "settings": self.settings,
                "exit_code": exit_code,
                "wall_elapsed_seconds": time.monotonic() - self.started_at if self.started_at else 0.0,
            })
            self.status = "completed" if exit_code == 0 and result.get("status") == "completed" else result.get("status", "failed")
            self.result_pub.publish(self._message(result))
            self._publish_state()
            self.process = None
            self.started_at = None

        def destroy_node(self):
            if self.process is not None and self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
            return super().destroy_node()

    rclpy.init()
    node = SpeedyDoubleLauncher()
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
