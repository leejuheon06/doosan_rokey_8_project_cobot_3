
# -*- coding: utf-8 -*-
"""Bootstrap the standalone two-robot H2017 palletizing simulation."""

from __future__ import annotations

import os
import sys
from pathlib import Path

# Kit은 --/app/fastShutdown=True로 뜬다. 하드 종료라 파이썬 stdout 버퍼가
# flush되지 않고 버려지므로, 파이프(tee)로 받으면 마지막 수천 줄이 통째로
# 사라진다. 줄 단위로 흘려보내면 flush=True를 매 print에 붙일 필요가 없다.
sys.stdout.reconfigure(line_buffering=True)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

# Configure Isaac Sim's bundled ROS2 libraries before the bridge starts. The
# dynamic loader reads LD_LIBRARY_PATH at process startup, so restart this same
# interpreter once after adding the bundled library directory.
ISAAC_ROOT = Path(sys.executable).parents[3]
ROS2_LIB_DIR = ISAAC_ROOT / "exts/isaacsim.ros2.bridge/humble/lib"
ROS2_PYTHON_DIR = ISAAC_ROOT / "exts/isaacsim.ros2.bridge/humble/rclpy"
if os.environ.get("H2017_ROS_ENV_READY") != "1":
    environment = os.environ.copy()
    environment.setdefault("ROS_DISTRO", "humble")
    environment.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    ld_library_path = environment.get("LD_LIBRARY_PATH", "")
    if str(ROS2_LIB_DIR) not in ld_library_path.split(":"):
        environment["LD_LIBRARY_PATH"] = ":".join(
            part for part in (ld_library_path, str(ROS2_LIB_DIR)) if part
        )
    environment["H2017_ROS_ENV_READY"] = "1"
    os.execve(
        sys.executable,
        [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]],
        environment,
    )

if str(ROS2_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(ROS2_PYTHON_DIR))

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": "--headless" in sys.argv})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.gen.conveyor")
for _ in range(30):
    simulation_app.update()

from h2017_palletizing.application import run

if __name__ == "__main__":
    try:
        run(simulation_app)
    except BaseException as exc:
        print(f"[실행 오류] {type(exc).__name__}: {exc}", flush=True)
        raise
    finally:
        simulation_app.close()
