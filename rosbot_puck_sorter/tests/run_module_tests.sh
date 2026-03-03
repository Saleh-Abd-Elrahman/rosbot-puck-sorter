#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TESTS=(
  "test_start_frame_manager.py"
  "test_aruco_home_mapper.py"
  "test_rgbd_puck_detector.py"
  "test_puck_world_model.py"
  "test_gripper_controller.py"
  "test_coverage_search.py"
  "test_pick_place_server.py"
  "test_mission_manager.py"
)

# NOTE: test_aruco_live_camera.py is hardware-in-the-loop and intentionally
# excluded from this automated list. Run it manually when camera + real markers are present.

if ! command -v roscore >/dev/null 2>&1; then
  echo "ERROR: roscore not found in PATH"
  exit 1
fi

for test_file in "${TESTS[@]}"; do
  echo "===== Running ${test_file} ====="

  roscore >/tmp/roscore_${test_file}.log 2>&1 &
  ROSCORE_PID=$!

  # Wait for roscore to come up.
  for _ in {1..40}; do
    if rosparam list >/dev/null 2>&1; then
      break
    fi
    sleep 0.25
  done

  set +e
  python3 "${SCRIPT_DIR}/${test_file}"
  status=$?
  set -e

  kill "${ROSCORE_PID}" >/dev/null 2>&1 || true
  wait "${ROSCORE_PID}" 2>/dev/null || true

  if [[ ${status} -ne 0 ]]; then
    echo "FAIL: ${test_file}"
    exit ${status}
  fi

  echo "PASS: ${test_file}"
  echo
  sleep 0.3
 done

echo "All module tests passed."
