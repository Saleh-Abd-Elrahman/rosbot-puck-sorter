# ROSbot Puck Sorter

This repo has been reset to the simplest challenge runner possible.

Active behavior:

- red puck -> ArUco marker ID 1
- green puck -> ArUco marker ID 2
- blue puck -> ArUco marker ID 3
- camera: `/camera/color/image_2fps/compressed`
- base command: `/cmd_vel`
- gripper command: `/servo`
- optional front safety from `/scan`

There is one ROS node:

- `rosbot_puck_sorter/scripts/pick_and_place.py`

## Run

Start the robot base, camera, and gripper rosserial first:

```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

Then:

```bash
catkin_make
source devel/setup.bash
roslaunch rosbot_puck_sorter mission.launch
```

## Notes

The node uses simple HSV blob tracking for pucks and ArUco detection for marker IDs. It does not use AMCL, odom, maps, waypoints, custom messages, action servers, or services.

It also publishes an approximate top-down path image at:

```bash
/pick_and_place/path_image
```

Open it with:

```bash
rqt_image_view /pick_and_place/path_image
```

If the robot computer has a display, it also opens a `Path Tracking` window.
