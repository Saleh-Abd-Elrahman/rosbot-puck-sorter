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
- Odometry (`/odom` plus `odom -> base_link` TF). AMCL is optional.
- RGB-D camera driver
- LiDAR driver
- ROSbot base driver that accepts `geometry_msgs/Twist` on `/cmd_vel`
- Arduino Nano running `rosserial` for gripper control
- `rosserial_python`

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
- odometry (`/odom` plus `odom -> base_link` TF); AMCL is optional
- base driver subscribed to `/cmd_vel`
- `rosrun rosserial_python serial_node.py /dev/ttyUSB0` for the gripper Arduino

Then:

```bash
roslaunch rosbot_puck_sorter mission.launch
```

If your ROSbot does not publish `/odom`, you can run a last-resort command
dead-reckoner instead:

```bash
roslaunch rosbot_puck_sorter mission.launch enable_cmd_vel_dead_reckoner:=true
```

That makes the package integrate its own `/cmd_vel` commands into an approximate
`/odom`. Real wheel odometry is still preferred.

## Configuration

Tune these files before first real run:
- `rosbot_puck_sorter/config/qr_home_mapper.yaml` (corner waypoints)
- `rosbot_puck_sorter/config/puck_color_hsv.yaml` (color thresholds)
- `rosbot_puck_sorter/config/coverage_search.yaml` (arena bounds)
- `rosbot_puck_sorter/config/gripper.yaml` (`rosserial` topics + gripper angles)
- `rosbot_puck_sorter/config/start_frame.yaml` (startup-relative `start` frame)

For ArUco markers, set `aruco_dictionary` and `aruco_id_to_color` in
`rosbot_puck_sorter/config/qr_home_mapper.yaml`.

For the gripper:
- upload [gripper_rosserial.ino](/Users/salehabdelrahman/Desktop/Rob_Lab_Proj/arduino/gripper_rosserial/gripper_rosserial.ino) to the Arduino Nano
- wire the current/load sensor to Arduino `A0` for the default pickup verification
- run `rosrun rosserial_python serial_node.py /dev/ttyUSB0`
- keep `backend: rosserial_topic` in `rosbot_puck_sorter/config/gripper.yaml`

## Tests

Run module-level tests:

```bash
cd ~/catkin_ws/src/<repo>/rosbot_puck_sorter/tests
./run_module_tests.sh
```

## Project docs

Detailed package-level documentation (nodes, topics, services, actions) is here:
- [rosbot_puck_sorter/README.md](rosbot_puck_sorter/README.md)
