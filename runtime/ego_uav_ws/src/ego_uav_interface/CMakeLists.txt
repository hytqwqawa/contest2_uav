cmake_minimum_required(VERSION 3.0.2)

project(ego_uav_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  mavros_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  state_monitor_node
  src/state_monitor_node.cpp
)

target_link_libraries(
  state_monitor_node
  ${catkin_LIBRARIES}
)

add_executable(
  preflight_monitor_node
  src/preflight_monitor_node.cpp
)

target_link_libraries(
  preflight_monitor_node
  ${catkin_LIBRARIES}
)


add_executable(
  offboard_hold_node
  src/offboard_hold_node.cpp
)

target_link_libraries(
  offboard_hold_node
  ${catkin_LIBRARIES}
)
