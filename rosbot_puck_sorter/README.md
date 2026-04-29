# rosbot_puck_sorter (ROS 1 Noetic)

ROS 1 package for color puck sorting with:
- reactive no-AMCL challenge runner for a 210x280 cm arena
- RGB-D puck detection for red, green, and blue pucks
- visual ArUco home finding (`ID 1 -> red`, `ID 2 -> green`, `ID 3 -> blue`)
- servo gripper control through an Arduino Nano over `rosserial`
- direct `/cmd_vel` driving with front wall safety from `/scan` and/or depth

## Package contents

- `scripts/challenge_manager.py`: main reactive challenge runner used by `mission.launch`
- `scripts/mission_manager.py`: legacy map/waypoint mission FSM
- `scripts/start_frame_manager.py`: captures initial odom/pose and publishes start-relative frame/topics
- `scripts/cmd_vel_dead_reckoner.py`: optional fallback odom/TF publisher from commanded velocity
- `scripts/startup_survey.py`: startup 360 rotation scan for ArUco homes + puck observations
- `scripts/semantic_grid_mapper.py`: builds start-frame occupancy grid + semantic object layer
- `scripts/qr_home_mapper.py`: scans 3 corners and maps home colors (ArUco or QR fallback)
- `scripts/rgbd_puck_detector.py`: HSV + depth based puck detection
- `scripts/puck_world_model.py`: tracking, target reservation, lifecycle
- `scripts/coverage_search.py`: rectangle coverage pass executor
- `scripts/fine_align_controller.py`: close-range camera alignment
- `scripts/pick_place_server.py`: one puck pick/place action server
- `scripts/gripper_controller.py`: gripper service node that publishes servo angles to `rosserial` topics

## Interfaces

### Published topics
- `/puck/detections` (`rosbot_puck_sorter/PuckDetectionArray`)
- `/mission/state` (`rosbot_puck_sorter/MissionState`)
- `/cmd_vel`
- `/gripper/state`, `/gripper/holding_object`

Legacy nodes can also publish:
- `/puck/tracks` (`rosbot_puck_sorter/PuckTrackArray`)
- `/puck/tracks_start` (`rosbot_puck_sorter/PuckTrackArray`, frame_id=`start`)
- `/home_bases` (`rosbot_puck_sorter/HomeBaseArray`)
- `/home_bases_start` (`rosbot_puck_sorter/HomeBaseArray`, frame_id=`start`)
- `/mission/current_target` (`rosbot_puck_sorter/PuckTrack`)
- `/start_frame/initialized` (`std_msgs/Bool`)
- `/start_frame/origin_map` (`geometry_msgs/PoseStamped`)
- `/robot_pose_start` (`geometry_msgs/PoseStamped`, frame_id=`start`)
- `/startup_survey/homes_start` (`rosbot_puck_sorter/HomeBaseArray`, frame_id=`start`)
- `/startup_survey/pucks_start` (`rosbot_puck_sorter/PuckTrackArray`, frame_id=`start`)
- `/semantic_map/grid` (`nav_msgs/OccupancyGrid`, frame_id=`start`)
- `/semantic_map/homes` (`rosbot_puck_sorter/HomeBaseArray`, frame_id=`start`)
- `/semantic_map/pucks` (`rosbot_puck_sorter/PuckTrackArray`, frame_id=`start`)
- `/coverage_search/pass_count`

### Services
- `/gripper/set` (`rosbot_puck_sorter/SetGripper`)

Legacy nodes can also expose:
- `/scan_homes` (`rosbot_puck_sorter/ScanHomes`)
- `/startup_survey/run` (`std_srvs/Trigger`)
- `/semantic_map/rebuild` (`std_srvs/Trigger`)
- `/puck_world_model/reserve_target` (`rosbot_puck_sorter/ReserveTarget`)
- `/coverage_search/perform_pass` (`std_srvs/Trigger`)
- `/fine_align/execute` (`std_srvs/Trigger`)

### Actions
- `/pick_and_place` (`rosbot_puck_sorter/PickAndPlaceAction`)

## External stack expected

Launch these separately (robot-specific):
- camera driver publishing the class lab topics (`/camera/color/image_2fps` and `/camera/depth/image_2fps`)
- LiDAR `/scan` if available; depth image safety is also used
- base motor driver (subscribes to `/cmd_vel`)
- Arduino Nano gripper firmware from [gripper_rosserial.ino](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/arduino/gripper_rosserial/gripper_rosserial.ino)

`move_base`, AMCL, `/odom`, and corner waypoints are not required for the main challenge runner. The robot searches in place, approaches visible pucks/markers with camera/depth feedback, and never drives to hard-coded arena coordinates.

## Build

From your catkin workspace root:

```bash
catkin_make
source devel/setup.bash
```

If this package directory is not under `<ws>/src`, move or symlink it there first.

If you still want the older direct-serial backend:

```bash
sudo apt install python3-serial
```

## Run

```bash
roslaunch rosbot_puck_sorter mission.launch
```

The mission launch starts:
- `rgbd_puck_detector.py`
- `gripper_controller.py`
- `challenge_manager.py`

It does not start the old home-scan, coverage, world-model, or pick/place action stack.

## Marker setup (ArUco)

- `config/challenge_manager.yaml` maps marker IDs to colors:
  - `"1": red`
  - `"2": green`
  - `"3": blue`
- Set `aruco_dictionary` to your marker family, for example `DICT_4X4_50`.
- Set `marker_size_m` to the real printed marker side length in meters (for distance estimation).
- Distance uses camera intrinsics from `camera_info_topic` by default.

## Gripper Setup (Professor `rosserial` workflow)

- Upload [gripper_rosserial.ino](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/arduino/gripper_rosserial/gripper_rosserial.ino) to the Nano.
- Generate `ros_lib` for the Arduino IDE if your setup does not already have it.
- Wire the servo signal to pin `9` and the current/load sensor to `A0`; the default pickup verification uses this load feedback.
- Re-upload this firmware after changes; `/servoLoad` is published in milliamps to match `load_feedback_threshold_ma`.
- On the robot computer connected to the Nano, run:

```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

- The Nano exposes:
  - `/servo` (`std_msgs/UInt16`) for angle commands
  - `/servoLoad` (`std_msgs/Float32`) for reported load in milliamps
- This package now defaults to:
  - `backend: rosserial_topic`
  - `servo_topic: /servo`
  - `servo_load_topic: /servoLoad`
  - `open_angle_deg: 0`
  - `close_angle_deg: 170`
- `/gripper/holding_object` reports a confirmed object hold from load feedback by default, unless `use_load_feedback` is disabled and `assume_holding_without_feedback` is explicitly set to `true`.
- The rest of the sorter still uses `/gripper/set`, so mission code does not need to change.
- You can also test the hardware directly with:

```bash
rostopic pub /servo std_msgs/UInt16 "data: 0" -1
rostopic pub /servo std_msgs/UInt16 "data: 170" -1
rostopic echo /servoLoad
```

## Main Challenge Behavior

`challenge_manager.py` runs one color at a time:

1. Rotate in place until a not-yet-delivered puck color is visible.
2. Use the puck's camera-frame `x` and depth to center and approach it.
3. Close the gripper through `/gripper/set`.
4. Rotate in place until the matching ArUco marker is visible.
5. Approach the marker to `marker_place_distance_m`.
6. Open the gripper and retreat.

It does not need pre-known home poses. A home is the matching ArUco marker currently visible in the camera.

## Tuning checklist

1. Tune HSV thresholds in `config/puck_color_hsv.yaml` under your actual lighting.
2. Set `marker_size_m`, `aruco_dictionary`, and `marker_id_to_color` in `config/challenge_manager.yaml`.
3. Tune `pickup_target_depth_m` and `marker_place_distance_m` in `config/challenge_manager.yaml`.
4. Keep `front_stop_distance_m` conservative until wall safety is proven.
5. Set `config/gripper.yaml` for the `rosserial` topics and calibrate `open_angle_deg` / `close_angle_deg`.
6. Verify the ROSbot base driver is subscribed to `/cmd_vel`.

## Legacy Stack

The older map/waypoint stack is still in the repo for reference and tests:
`mission_manager.py`, `pick_place_server.py`, `coverage_search.py`,
`qr_home_mapper.py`, `startup_survey.py`, `puck_world_model.py`, and
`semantic_grid_mapper.py`. It is not launched by `mission.launch`.

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

If your marker IDs differ, update `_expected_ids` and `config/challenge_manager.yaml` (`marker_id_to_color`).

## Notes

- Red uses two HSV hue ranges (`red1`, `red2`) due hue wraparound in OpenCV.
- The main challenge runner does not store map-frame home poses.
- Gripper hold detection uses load feedback by default; tune `load_feedback_threshold_ma` in [gripper.yaml](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/rosbot_puck_sorter/config/gripper.yaml) after checking `/servoLoad`.
