# rosbot_puck_sorter

This package now contains the lab reference Python scripts copied from:

- `/Users/salehabdelrahman/Downloads/robotics-lab-final-master.zip`

The old custom sorter stack has been removed from the active package.

## Active Launch

```bash
roslaunch rosbot_puck_sorter mission.launch
```

This launches only:

- `scripts/grab_and_deliver.py`

That script uses the same topics and constants as the reference code:

- `/camera/color/image_2fps/compressed`
- `/cmd_vel`
- `/servo`
- red puck only
- ArUco marker ID `1`

## Scripts

The copied scripts are in `scripts/`:

- `detect_pucks.py`
- `detect_pucks_distance.py`
- `follow_red_puck.py`
- `grab_red_puck.py`
- `grab_and_deliver.py`
- `gripper_reset.py`
- plus the other exercise/debug scripts from the zip
