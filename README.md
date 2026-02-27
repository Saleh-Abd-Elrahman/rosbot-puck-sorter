# ROSbot Puck Sorter (ROS 1 Noetic)

This repository contains a ROS 1 project for a ROSbot that:
- scans QR-coded home bases (`red`, `green`, `blue`) at arena corners
- detects colored pucks with an RGB-D camera
- picks and places each puck into the matching home using a servo gripper
- keeps surveying while running to determine when sorting is complete

The ROS package is in:
- `rosbot_puck_sorter/`

## Requirements

- Ubuntu 20.04
- ROS 1 Noetic
- Catkin workspace
- Navigation stack (`move_base`)
- Localization (`amcl`)
- RGB-D camera driver
- LiDAR driver
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

Launch your robot bringup (camera/lidar/nav/amcl/base driver) first, then:

```bash
roslaunch rosbot_puck_sorter mission.launch
```

## Configuration

Tune these files before first real run:
- `rosbot_puck_sorter/config/qr_home_mapper.yaml` (corner waypoints)
- `rosbot_puck_sorter/config/puck_color_hsv.yaml` (color thresholds)
- `rosbot_puck_sorter/config/coverage_search.yaml` (arena bounds)
- `rosbot_puck_sorter/config/gripper.yaml` (servo PWM + open/close angles)

## Project docs

Detailed package-level documentation (nodes, topics, services, actions) is here:
- [rosbot_puck_sorter/README.md](rosbot_puck_sorter/README.md)
