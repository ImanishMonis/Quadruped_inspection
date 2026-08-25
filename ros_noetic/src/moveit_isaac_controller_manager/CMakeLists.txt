cmake_minimum_required(VERSION 3.0.2)

project(moveit_isaac_controller_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  moveit_core
  moveit_ros_planning
  trajectory_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_isaac_controller_manager
  CATKIN_DEPENDS
    roscpp
    pluginlib
    moveit_core
    moveit_ros_planning
    trajectory_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(moveit_isaac_controller_manager

  src/isaac_controller.cpp
  src/isaac_controller_manager.cpp
)

target_link_libraries(
  moveit_isaac_controller_manager
  ${catkin_LIBRARIES}
)

install(
  TARGETS moveit_isaac_controller_manager
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)