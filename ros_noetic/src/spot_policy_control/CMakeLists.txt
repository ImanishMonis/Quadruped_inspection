cmake_minimum_required(VERSION 3.0.2)
project(spot_policy_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/spot_ros_controller.py
  scripts/spot_teleop.py
  scripts/spot_move_relative.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
