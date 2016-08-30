cmake_minimum_required(VERSION 2.8.3)
project(rviz_markers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rviz_markers src/rviz_markers.cpp)
target_link_libraries(rviz_markers ${catkin_LIBRARIES})