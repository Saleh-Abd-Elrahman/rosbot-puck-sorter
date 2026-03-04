cmake_minimum_required(VERSION 3.0.2)
project(rosbot_puck_sorter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  message_filters
  actionlib
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  PuckDetection.msg
  PuckDetectionArray.msg
  PuckTrack.msg
  PuckTrackArray.msg
  HomeBase.msg
  HomeBaseArray.msg
  MissionState.msg
)

add_service_files(
  FILES
  ScanHomes.srv
  SetGripper.srv
  ReserveTarget.srv
)

add_action_files(
  FILES
  PickAndPlace.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    move_base_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    message_filters
    actionlib
    actionlib_msgs
    message_runtime
)

catkin_install_python(PROGRAMS
  scripts/mission_manager.py
  scripts/start_frame_manager.py
  scripts/startup_survey.py
  scripts/semantic_grid_mapper.py
  scripts/qr_home_mapper.py
  scripts/rgbd_puck_detector.py
  scripts/puck_world_model.py
  scripts/coverage_search.py
  scripts/pick_place_server.py
  scripts/gripper_controller.py
  scripts/fine_align_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config msg srv action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
