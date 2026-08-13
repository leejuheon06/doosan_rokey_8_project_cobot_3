#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCRIPT_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

python3 -m conveyor_box_measurement.node \
  --ros-args \
  -r __node:=conveyor_1_measurement \
  --params-file "$SCRIPT_DIR/config/measurement_conveyor_1.yaml" &
VISION_PID_1=$!

python3 -m conveyor_box_measurement.node \
  --ros-args \
  -r __node:=conveyor_2_measurement \
  --params-file "$SCRIPT_DIR/config/measurement_conveyor_2.yaml" &
VISION_PID_2=$!

cleanup() {
  kill "$VISION_PID_1" "$VISION_PID_2" 2>/dev/null || true
  wait "$VISION_PID_1" "$VISION_PID_2" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "$VISION_PID_1" "$VISION_PID_2"
