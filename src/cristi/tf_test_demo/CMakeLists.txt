cmake_minimum_required(VERSION 2.8.3)
project(tf_test_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_demo src/tf_demo.cpp)
target_link_libraries(tf_demo
  ${catkin_LIBRARIES}
)