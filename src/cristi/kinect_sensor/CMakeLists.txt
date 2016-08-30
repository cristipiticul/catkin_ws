cmake_minimum_required(VERSION 2.8.3)
project(kinect_sensor)

find_package(PCL REQUIRED COMPONENTS common io features visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  openni2_launch
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cloud_viewer_ros
  src/cloud_viewer_ros.cpp
)

target_link_libraries(cloud_viewer_ros
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

add_executable(simulated_cloud_viewer
  src/simulated_cloud_viewer.cpp
)

target_link_libraries(simulated_cloud_viewer
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

