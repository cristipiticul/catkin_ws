cmake_minimum_required(VERSION 2.8.3)
project(pick_place_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules dynamic_reconfigure)

find_package(Boost REQUIRED system filesystem date_time thread)


generate_dynamic_reconfigure_options(
  cfg/Position.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    dynamic_reconfigure
)


find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#add_subdirectory(kinematics)
#add_subdirectory(planning)
#add_subdirectory(state_display)
#add_subdirectory(interactivity)
#add_subdirectory(pick_place)


add_executable(pick_place_demo src/pose_reconfigure.cpp)
target_link_libraries(pick_place_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(pick_place_demo ${PROJECT_NAME}_gencfg)

