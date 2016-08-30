cmake_minimum_required(VERSION 2.8.3)
project(octomap_getting_started)

find_package(catkin REQUIRED)
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

catkin_package(

)

add_executable(example src/example.cpp)
target_link_libraries(example ${CATKIN_LIBRARIES})

