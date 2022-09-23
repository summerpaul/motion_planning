cmake_minimum_required(VERSION 3.0.2)
project(map_generator)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  roscpp
  rospy
  std_msgs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(random_map STATIC src/random_map.cpp)
target_link_libraries(random_map
          ${catkin_LIBRARIES}
          ${PCL_LIBRARIES})
add_executable(random_map_node src/main.cpp)
target_link_libraries(random_map_node random_map)          