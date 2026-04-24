# rosbot_puck_sorter (ROS 1 Noetic)

ROS 1 package for color puck sorting with:
- ArUco-marker-based dynamic home mapping (`red`, `green`, `blue`)
- Continuous RGB-D puck detection and world tracking
- Pick/place action with servo gripper control through an Arduino Nano over USB serial
- Coverage search when no valid targets are available

## Package contents

- `scripts/mission_manager.py`: mission FSM and orchestration
- `scripts/start_frame_manager.py`: captures initial pose and publishes start-relative frame/topics
- `scripts/startup_survey.py`: startup 360 rotation scan for ArUco homes + puck observations
- `scripts/semantic_grid_mapper.py`: builds start-frame occupancy grid + semantic object layer
- `scripts/qr_home_mapper.py`: scans 3 corners and maps home colors (ArUco or QR fallback)
- `scripts/rgbd_puck_detector.py`: HSV + depth based puck detection
- `scripts/puck_world_model.py`: tracking, target reservation, lifecycle
- `scripts/coverage_search.py`: rectangle coverage pass executor
- `scripts/fine_align_controller.py`: close-range camera alignment
- `scripts/pick_place_server.py`: one puck pick/place action server
- `scripts/gripper_controller.py`: gripper service node that sends serial commands to the Arduino Nano

## Interfaces

### Published topics
- `/puck/detections` (`rosbot_puck_sorter/PuckDetectionArray`)
- `/puck/tracks` (`rosbot_puck_sorter/PuckTrackArray`)
- `/puck/tracks_start` (`rosbot_puck_sorter/PuckTrackArray`, frame_id=`start`)
- `/home_bases` (`rosbot_puck_sorter/HomeBaseArray`)
- `/home_bases_start` (`rosbot_puck_sorter/HomeBaseArray`, frame_id=`start`)
- `/mission/state` (`rosbot_puck_sorter/MissionState`)
- `/mission/current_target` (`rosbot_puck_sorter/PuckTrack`)
- `/start_frame/initialized` (`std_msgs/Bool`)
- `/start_frame/origin_map` (`geometry_msgs/PoseStamped`)
- `/robot_pose_start` (`geometry_msgs/PoseStamped`, frame_id=`start`)
- `/startup_survey/homes_start` (`rosbot_puck_sorter/HomeBaseArray`, frame_id=`start`)
- `/startup_survey/pucks_start` (`rosbot_puck_sorter/PuckTrackArray`, frame_id=`start`)
- `/semantic_map/grid` (`nav_msgs/OccupancyGrid`, frame_id=`start`)
- `/semantic_map/homes` (`rosbot_puck_sorter/HomeBaseArray`, frame_id=`start`)
- `/semantic_map/pucks` (`rosbot_puck_sorter/PuckTrackArray`, frame_id=`start`)
- `/cmd_vel_align`, `/cmd_vel_nav`
- `/coverage_search/pass_count`
- `/gripper/state`, `/gripper/holding_object`

### Services
- `/scan_homes` (`rosbot_puck_sorter/ScanHomes`)
- `/startup_survey/run` (`std_srvs/Trigger`)
- `/semantic_map/rebuild` (`std_srvs/Trigger`)
- `/puck_world_model/reserve_target` (`rosbot_puck_sorter/ReserveTarget`)
- `/coverage_search/perform_pass` (`std_srvs/Trigger`)
- `/fine_align/execute` (`std_srvs/Trigger`)
- `/gripper/set` (`rosbot_puck_sorter/SetGripper`)

### Actions
- `/pick_and_place` (`rosbot_puck_sorter/PickAndPlaceAction`)

## External stack expected

Launch these separately (robot-specific):
- camera driver (RGB-D topics)
- LiDAR driver
- localization (`amcl`)
- nav stack (`move_base`)
- base motor driver (subscribes to `/cmd_vel`)
- `twist_mux` configured to arbitrate `/cmd_vel_nav` and `/cmd_vel_align`
- Arduino Nano gripper firmware from [gripper_serial_bridge.ino](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/arduino/gripper_serial_bridge/gripper_serial_bridge.ino)

## Build

From your catkin workspace root:

```bash
catkin_make
source devel/setup.bash
```

If this package directory is not under `<ws>/src`, move or symlink it there first.

Install serial dependency if needed:

```bash
sudo apt install python3-serial
```

## Run

```bash
roslaunch rosbot_puck_sorter mission.launch
```

## Marker setup (ArUco)

- `config/qr_home_mapper.yaml` now defaults to `marker_mode: aruco`.
- Set `aruco_dictionary` to your marker family (for example `DICT_4X4_50`).
- Set `aruco_id_to_color` so each marker id maps to one home color:
  - `\"0\": red`
  - `\"1\": green`
  - `\"2\": blue`
- Set `marker_size_m` to the real printed marker side length in meters (for distance estimation).
- Distance uses camera intrinsics from `camera_info_topic` by default (`use_camera_info_intrinsics: true`).
  - You can override with `fx`, `fy`, `cx`, `cy` params.
  - If intrinsics are unavailable, fallback estimation uses `hfov_deg`.
- QR payload mode is still available by setting `marker_mode: qr` and using `qr_expected_codes`.

## Gripper Setup (Arduino Nano over USB)

- Upload [gripper_serial_bridge.ino](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/arduino/gripper_serial_bridge/gripper_serial_bridge.ino) to the Nano.
- Wire the servo signal to the pin configured in the sketch (`SERVO_PIN`, default `9`).
- Set the matching USB device in `config/gripper.yaml`:
  - `backend: arduino_serial`
  - `serial_port: /dev/ttyUSB0` or `/dev/ttyACM0`
  - `serial_baud_rate: 115200`
- `open_angle_deg` and `close_angle_deg` are controlled from the ROS side and sent as `ANGLE <deg>` commands over serial.
- The Nano replies with simple ASCII acknowledgements:
  - `OK ANGLE 20`
  - `ERR ...`
- Hold detection is still heuristic on the ROS side unless you add a real sensor.

## Startup Survey Behavior

- At mission start, `mission_manager` calls `/startup_survey/run` first.
- `startup_survey` rotates in place ~360 degrees and:
  - detects ArUco homes and stores start-frame relative position + distance
  - collects puck detections during the same rotation and stores start-frame positions
- Results are published on:
  - `/startup_survey/homes_start`
  - `/startup_survey/pucks_start`
- Snapshot is saved to:
  - `~/.ros/rosbot_puck_sorter_startup_survey.json` (configurable in `config/startup_survey.yaml`)

## Target Selection (Efficiency-First)

- Mission manager uses cost-optimal selection (default `target_policy: cost_optimal`).
- One-puck-per-color mode is enabled by default (`single_puck_per_color: true`), with expected colors from `expected_colors: [red, green, blue]`.
- Once a color is successfully delivered, that color is marked complete and no further puck of that color is selected.
- For each candidate puck, score is based on:
  - estimated `robot -> puck` cost
  - estimated `puck -> corresponding_home` cost
  - penalties (low confidence, prior failures, wall proximity, turn effort)
- Optional one-step lookahead evaluates likely next cycle cost and biases the first choice.
- If `/move_base/make_plan` is available, path length is used; otherwise Euclidean fallback is used.
- Dynamic replanning is applied after every completed drop (next best puck is recomputed).
- Coverage search only triggers after no viable target is available for a timeout window (`search_trigger_s`).
- Mission ends when all `expected_colors` are delivered (strict mode), or by empty-area verification if strict mode is disabled.

## Semantic Map (Option 2)

- `semantic_grid_mapper` subscribes to startup survey outputs (and optionally dynamic puck tracks) and publishes:
  - occupancy grid (`/semantic_map/grid`)
  - home object layer (`/semantic_map/homes`)
  - puck object layer (`/semantic_map/pucks`)
- Grid is defined in `start` frame using configurable rectangle bounds and resolution.
- Objects are painted into the grid while preserving dedicated semantic topics for exact object identities.
- Tune in `config/semantic_map.yaml`:
  - grid bounds/resolution
  - wall/home/puck paint radii and occupancy values
  - dynamic puck tracking behavior

## Tuning checklist

1. Update `config/qr_home_mapper.yaml` corner waypoints to your arena map coordinates.
2. Tune HSV thresholds in `config/puck_color_hsv.yaml` under your actual lighting.
3. Tune coverage bounds in `config/coverage_search.yaml`.
4. Set `config/gripper.yaml` for the Nano serial port and calibrate `open_angle_deg` / `close_angle_deg`.
5. Verify `twist_mux` wiring and ensure only mux output goes to robot `/cmd_vel`.
6. Keep `config/start_frame.yaml` enabled if you want startup-relative coordinates.
7. Tune `config/startup_survey.yaml` (rotation speed, marker size, snapshot path).
8. Tune `config/semantic_map.yaml` (occupancy grid bounds/resolution/object painting).
9. Tune `config/mission_manager.yaml` selection weights (`lookahead`, penalties, `search_trigger_s`, `use_make_plan_cost`).

## Module Tests

Test scripts are in `tests/` and cover modules individually:
- `test_start_frame_manager.py`
- `test_aruco_home_mapper.py`
- `test_aruco_live_camera.py` (real camera + real ArUco markers; no synthetic image)
- `test_rgbd_puck_detector.py`
- `test_puck_world_model.py`
- `test_gripper_controller.py`
- `test_coverage_search.py`
- `test_semantic_grid_mapper.py`
- `test_pick_place_server.py`
- `test_mission_manager.py`

Run all module tests:

```bash
cd rosbot_puck_sorter/tests
./run_module_tests.sh
```

Run the real-camera ArUco test manually:

```bash
roscore
# in another terminal:
rosrun rosbot_puck_sorter test_aruco_live_camera.py _image_topic:=/camera/color/image_raw _aruco_dictionary:=DICT_4X4_50 _expected_ids:=\"[0,1,2]\" _stable_reads_required:=8 _timeout_s:=30
```

If your marker IDs differ, update `_expected_ids` and `config/qr_home_mapper.yaml` (`aruco_id_to_color`).

## Notes

- Red uses two HSV hue ranges (`red1`, `red2`) due hue wraparound in OpenCV.
- Home bases are stored as full map-frame `Pose` (`x`, `y`, `z` + orientation).
- `HomeBase` now includes `marker_distance_m` from ArUco pose estimation (meters; `-1.0` when unavailable).
- Navigation still runs in `map`; start-relative telemetry is published in `start`.
- Current gripper hold detection is heuristic (no force/current sensor). Add hardware feedback if available.
