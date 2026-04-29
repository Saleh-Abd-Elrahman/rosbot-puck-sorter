# ROSbot Puck Sorter (ROS 1 Noetic)

This repository contains a ROS 1 project for a ROSbot that:
- detects red, green, and blue pucks with an RGB-D camera
- grabs each puck with a rosserial-controlled servo gripper
- finds the matching ArUco marker visually (`ID 1 -> red`, `ID 2 -> green`, `ID 3 -> blue`)
- places the puck in front of that marker inside the yellow area
- drives only with direct `geometry_msgs/Twist` commands on `/cmd_vel`

The ROS package is in:
- `rosbot_puck_sorter/`

## Requirements

- Ubuntu 20.04
- ROS 1 Noetic
- Catkin workspace
- RGB-D camera driver
- LiDAR `/scan` is recommended for wall safety, but the RGB-D depth image also provides front safety
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
- RGB-D camera publishing the class lab topics (`/camera/color/image_2fps` and `/camera/depth/image_2fps`)
- optional LiDAR `/scan`
- base driver subscribed to `/cmd_vel`
- `rosrun rosserial_python serial_node.py /dev/ttyUSB0` for the gripper Arduino

Then:

```bash
roslaunch rosbot_puck_sorter mission.launch
```

The main launch is now reactive and no-AMCL: it does not use corner waypoints,
coverage waypoints, `/odom`, or `/amcl_pose`.

## Configuration

Tune these files before first real run:
- `rosbot_puck_sorter/config/challenge_manager.yaml` (challenge behavior, ArUco IDs, safety distances)
- `rosbot_puck_sorter/config/puck_color_hsv.yaml` (color thresholds)
- `rosbot_puck_sorter/config/gripper.yaml` (`rosserial` topics + gripper angles)

For ArUco markers, set `aruco_dictionary`, `marker_size_m`, and
`marker_id_to_color` in `rosbot_puck_sorter/config/challenge_manager.yaml`.

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
