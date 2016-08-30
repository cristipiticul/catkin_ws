cmake_minimum_required(VERSION 2.8.3)
project(detection)

find_package(Boost REQUIRED COMPONENTS system thread timer chrono)
find_package(PCL REQUIRED COMPONENTS common io features visualization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  eigen_conversions
)

generate_dynamic_reconfigure_options(cfg/Parameters.cfg)

find_package(Threads REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_msgs pcl_ros sensor_msgs std_msgs dynamic_reconfigure eigen_conversions
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(point_cloud_algorithms
	src/point_cloud_algorithms.cpp
)
add_dependencies(point_cloud_algorithms
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(point_cloud_algorithms
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

add_library(point_cloud_processing_pipeline
	src/point_cloud_processing_pipeline.cpp
)
add_dependencies(point_cloud_processing_pipeline
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(point_cloud_processing_pipeline
  point_cloud_algorithms
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

add_library(cylinder_detector
	src/cylinder_detector.cpp
)
add_dependencies(cylinder_detector
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(cylinder_detector
  point_cloud_algorithms
  point_cloud_processing_pipeline
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

add_library(color_detector
	src/color_detector.cpp
)
add_dependencies(color_detector
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(color_detector
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)



add_executable(bottle_detector
  src/bottle_detector.cpp
)
add_dependencies(bottle_detector
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(bottle_detector
  ${catkin_LIBRARIES}
	${PCL_COMMON_LIBRARIES}
	color_detector
	cylinder_detector
)
install(
	TARGETS bottle_detector
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
