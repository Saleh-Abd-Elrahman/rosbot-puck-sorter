# ROSbot Puck Sorter (ROS 1 Noetic)

This repository contains a ROS 1 project for a ROSbot that:
- scans ArUco-marked home bases (`red`, `green`, `blue`) at arena corners
- detects colored pucks with an RGB-D camera
- picks and places one puck per color into the matching home using a servo gripper
- keeps surveying while running to determine when sorting is complete
- runs a startup 360-degree in-place survey to capture initial home/puck observations
- builds a start-frame occupancy grid plus semantic object layer (`homes` and `pucks`)

The ROS package is in:
- `rosbot_puck_sorter/`

## Requirements

- Ubuntu 20.04
- ROS 1 Noetic
- Catkin workspace
- Localization (`amcl`)
- RGB-D camera driver
- LiDAR driver
- ROSbot base driver that accepts `geometry_msgs/Twist` on `/cmd_vel`
- Arduino Nano connected by USB serial to the ROSbot for gripper control
- `python3-serial`
- `twist_mux` (recommended for `/cmd_vel` arbitration)

## Build

From your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <your-repo-url>
cd ..
catkin_make
source devel/setup.bash
```

## Run

Launch your robot bringup first so these are already running:
- RGB-D camera
- LiDAR
- localization (`amcl`)
- base driver
- `twist_mux` or equivalent routing of `/cmd_vel_nav` and `/cmd_vel_align` to the robot `/cmd_vel`

Then:

```bash
roslaunch rosbot_puck_sorter mission.launch
```

## Configuration

Tune these files before first real run:
- `rosbot_puck_sorter/config/qr_home_mapper.yaml` (corner waypoints)
- `rosbot_puck_sorter/config/puck_color_hsv.yaml` (color thresholds)
- `rosbot_puck_sorter/config/coverage_search.yaml` (arena bounds)
- `rosbot_puck_sorter/config/gripper.yaml` (USB serial port + gripper angles)
- `rosbot_puck_sorter/config/start_frame.yaml` (startup-relative `start` frame)

For ArUco markers, set `aruco_dictionary` and `aruco_id_to_color` in
`rosbot_puck_sorter/config/qr_home_mapper.yaml`.

For the gripper:
- upload [gripper_serial_bridge.ino](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/arduino/gripper_serial_bridge/gripper_serial_bridge.ino) to the Arduino Nano
- set the Nano USB device path in `rosbot_puck_sorter/config/gripper.yaml` (`serial_port`)

## Tests

Run module-level tests:

```bash
cd ~/catkin_ws/src/<repo>/rosbot_puck_sorter/tests
./run_module_tests.sh
```

## Project docs

Detailed package-level documentation (nodes, topics, services, actions) is here:
- [rosbot_puck_sorter/README.md](rosbot_puck_sorter/README.md)
