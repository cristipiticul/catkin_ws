cmake_minimum_required(VERSION 2.8.3)
project(transforms_saver)

find_package(catkin REQUIRED COMPONENTS
  message_runtime
  roscpp
  std_msgs
  std_srvs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(saver src/saver.cpp)
target_link_libraries(saver
  ${catkin_LIBRARIES}
)

