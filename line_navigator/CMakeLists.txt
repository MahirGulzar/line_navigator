cmake_minimum_required(VERSION 3.0.2)
project(line_navigator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2_ros
  costmap_2d
  image_geometry
  geometry_msgs
  gazebo_msgs
  nav_msgs
  move_base_msgs
  actionlib
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  tf
  tf2_ros 
  costmap_2d    
  image_geometry
  geometry_msgs
  gazebo_msgs
  nav_msgs
  move_base_msgs
  actionlib
  std_msgs 
  sensor_msgs 
  cv_bridge 
  image_transport
  DEPENDS OpenCV
)

# Compile the lane_follower_node executable from both source files
add_executable(lane_follower_node src/lane_follower_node.cpp src/lane_follower.cpp)
add_executable(fake_pose_node src/fake_pose_node.cpp src/fake_pose.cpp)
# Correct target_link_libraries entry
target_link_libraries(lane_follower_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(fake_pose_node
  ${catkin_LIBRARIES}
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


install(
  TARGETS
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
