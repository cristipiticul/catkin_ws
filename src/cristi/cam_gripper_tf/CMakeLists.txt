cmake_minimum_required(VERSION 2.8.3)
project(cam_gripper_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${EIGEN3_LIBRARY_DIRS}
)

add_executable(get_transforms src/get_transforms.cpp)
target_link_libraries(get_transforms
   ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)