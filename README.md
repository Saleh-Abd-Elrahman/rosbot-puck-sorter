# ROSbot Lab Reference Scripts

This repo now runs the copied code from:

- `/Users/salehabdelrahman/Downloads/robotics-lab-final-master.zip`

The active launch runs:

- `rosbot_puck_sorter/scripts/grab_and_deliver.py`

That script is the lab reference behavior:

- find a red puck on `/camera/color/image_2fps/compressed`
- drive using `/cmd_vel`
- close/open the gripper by publishing `std_msgs/UInt16` directly to `/servo`
- find ArUco marker ID `1`
- drive to it and release the puck

## Run

Start your robot base, camera, and rosserial first:

```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

Then run:

```bash
roslaunch rosbot_puck_sorter mission.launch
```

## Useful Direct Tests

```bash
rosrun rosbot_puck_sorter detect_pucks.py
rosrun rosbot_puck_sorter puck_debug.py
rosrun rosbot_puck_sorter grab_red_puck.py
rosrun rosbot_puck_sorter grab_and_deliver.py
rosrun rosbot_puck_sorter gripper_reset.py open
```
