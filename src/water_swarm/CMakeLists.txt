cmake_minimum_required(VERSION 3.0.2)
project(water_swarm)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  common_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs roscpp rospy common_msgs
)

include_directories(
  include/water_swarm
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(sph_multi_swarm src/sph_multi_swarm.cpp)
target_link_libraries(sph_multi_swarm ${catkin_LIBRARIES})
add_dependencies(sph_multi_swarm ${catkin_EXPORTED_TARGETS}) 
