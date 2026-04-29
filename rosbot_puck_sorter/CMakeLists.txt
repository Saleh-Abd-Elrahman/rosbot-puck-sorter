cmake_minimum_required(VERSION 3.0.2)
project(rosbot_puck_sorter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/detect_pucks.py
  scripts/detect_pucks_distance.py
  scripts/exercise1.py
  scripts/exercise2.py
  scripts/follow_red_puck.py
  scripts/grab_and_deliver.py
  scripts/grab_red_puck.py
  scripts/gripper_reset.py
  scripts/ir_avoidance.py
  scripts/jose.py
  scripts/my_publisher.py
  scripts/puck_debug.py
  scripts/save_color_image.py
  scripts/simple_camera_sub.py
  scripts/simple_camera_sub_compressed.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
