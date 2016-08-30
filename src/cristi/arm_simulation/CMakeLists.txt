cmake_minimum_required(VERSION 2.8.3)
project(arm_simulation)

find_package(Boost REQUIRED COMPONENTS system thread timer chrono)
find_package(PCL REQUIRED COMPONENTS common io features visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_msgs pcl_ros sensor_msgs std_msgs
)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_corrector
  src/pointcloud_corrector.cpp
)
target_link_libraries(pointcloud_corrector
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
)
