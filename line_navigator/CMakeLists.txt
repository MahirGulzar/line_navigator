cmake_minimum_required(VERSION 3.0.2)
project(line_navigator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  costmap_2d
  gmapping
  turtlebot3_bringup
  turtlebot3_slam
  turtlebot3_example
  image_geometry
  move_base_msgs
  std_srvs
  actionlib
  tf
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  tf2_ros 
  costmap_2d 
  navigation 
  gmapping 
  turtlebot3_bringup 
  turtlebot3_slam 
  turtlebot3_example
  image_geometry
  move_base_msgs
  std_srvs
  actionlib
  tf
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
)


# Compile the lane_follower_node executable from both source files
add_executable(lane_follower_node src/lane_follower_node.cpp src/lane_follower.cpp)
# Correct target_link_libraries entry
target_link_libraries(lane_follower_node
  ${catkin_LIBRARIES}
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


install(
  TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
