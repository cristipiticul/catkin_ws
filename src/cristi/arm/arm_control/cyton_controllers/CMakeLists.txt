cmake_minimum_required(VERSION 2.8.3)
project(cyton_controllers)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_controllers
)

## For gripper interface
find_package(catkin REQUIRED
  genmsg
  actionlib_msgs
  actionlib
  dynamixel_msgs
  control_msgs
  std_msgs
)
generate_messages(DEPENDENCIES
  dynamixel_msgs
  control_msgs
  actionlib_msgs 
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# add_executable(griper_moveit_interface src/gripper_moveit_interface.cpp)
# target_link_libraries(griper_moveit_interface ${catkin_LIBRARIES})
