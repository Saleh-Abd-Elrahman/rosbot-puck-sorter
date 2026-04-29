# rosbot_puck_sorter

Minimal ROS 1 package for the final challenge.

It installs and launches only:

- `scripts/pick_and_place.py`

The node performs the challenge with direct camera servoing:

1. Find the red puck, grab it, place it at marker ID 1.
2. Find the green puck, grab it, place it at marker ID 2.
3. Find the blue puck, grab it, place it at marker ID 3.

Topics:

- subscribes: `/camera/color/image_2fps/compressed`
- subscribes: `/scan` if available for front stop safety
- publishes: `/cmd_vel`
- publishes: `/servo`

Run:

```bash
roslaunch rosbot_puck_sorter mission.launch
```
