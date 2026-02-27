# rosbot_puck_sorter (ROS 1 Noetic)

ROS 1 package for color puck sorting with:
- QR-based dynamic home mapping (`red`, `green`, `blue`)
- Continuous RGB-D puck detection and world tracking
- Pick/place action with servo gripper control over PWM
- Coverage search when no valid targets are available

## Package contents

- `scripts/mission_manager.py`: mission FSM and orchestration
- `scripts/start_frame_manager.py`: captures initial pose and publishes start-relative frame/topics
- `scripts/qr_home_mapper.py`: scans 3 corners and maps home colors
- `scripts/rgbd_puck_detector.py`: HSV + depth based puck detection
- `scripts/puck_world_model.py`: tracking, target reservation, lifecycle
- `scripts/coverage_search.py`: rectangle coverage pass executor
- `scripts/fine_align_controller.py`: close-range camera alignment
- `scripts/pick_place_server.py`: one puck pick/place action server
- `scripts/gripper_controller.py`: servo PWM gripper service node

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
- `/cmd_vel_align`, `/cmd_vel_nav`
- `/coverage_search/pass_count`
- `/gripper/state`, `/gripper/holding_object`

### Services
- `/scan_homes` (`rosbot_puck_sorter/ScanHomes`)
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

## Build

From your catkin workspace root:

```bash
catkin_make
source devel/setup.bash
```

If this package directory is not under `<ws>/src`, move or symlink it there first.

## Run

```bash
roslaunch rosbot_puck_sorter mission.launch
```

## Tuning checklist

1. Update `config/qr_home_mapper.yaml` corner waypoints to your arena map coordinates.
2. Tune HSV thresholds in `config/puck_color_hsv.yaml` under your actual lighting.
3. Tune coverage bounds in `config/coverage_search.yaml`.
4. Calibrate gripper servo angles and pulse range in `config/gripper.yaml`.
5. Verify `twist_mux` wiring and ensure only mux output goes to robot `/cmd_vel`.
6. Keep `config/start_frame.yaml` enabled if you want startup-relative coordinates.

## Notes

- Red uses two HSV hue ranges (`red1`, `red2`) due hue wraparound in OpenCV.
- Home bases are stored as full map-frame `Pose` (`x`, `y`, `z` + orientation).
- Navigation still runs in `map`; start-relative telemetry is published in `start`.
- Current gripper hold detection is heuristic (no force/current sensor). Add hardware feedback if available.
