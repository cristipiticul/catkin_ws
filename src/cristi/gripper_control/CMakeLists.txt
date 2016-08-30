cmake_minimum_required(VERSION 2.8.3)
project(gripper_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gencpp
  message_generation
)

add_service_files(
  FILES
  GripperCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

add_library(gripper_command_listener src/gripper_command_listener.cpp)

add_executable(gripper_control_sim src/gripper_control_sim.cpp)
add_executable(gripper_control_real src/gripper_control_real.cpp)

add_dependencies(gripper_control_sim gripper_control_generate_messages_cpp)
add_dependencies(gripper_control_real gripper_control_generate_messages_cpp)

target_link_libraries(gripper_control_sim
  ${catkin_LIBRARIES} gripper_command_listener
)
target_link_libraries(gripper_control_real
  ${catkin_LIBRARIES} gripper_command_listener
)
