cmake_minimum_required(VERSION 2.8.3)
project(cyton_gamma_1500_moveit_conf)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	moveit_ros_move_group
	moveit_planners_ompl
	moveit_ros_visualization
	joint_state_publisher
	robot_state_publisher
	xacro
	cyton_gamma_1500_description
)
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(move_arm src/move_arm.cpp)
target_link_libraries(move_arm
  ${catkin_LIBRARIES}
)

add_executable(test_coordinates src/test_coordinates.cpp)
target_link_libraries(test_coordinates
  ${catkin_LIBRARIES}
)

add_executable(move_gripper src/move_gripper.cpp)
target_link_libraries(move_gripper
  ${catkin_LIBRARIES}
)
