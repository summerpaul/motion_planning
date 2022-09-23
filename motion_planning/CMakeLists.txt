cmake_minimum_required(VERSION 3.0.2)
project(motion_planning_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  pcl_ros
  ros_viz_tools
  interactive_markers
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_planning_ros
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  core
  ros_test
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_subdirectory(core)
add_subdirectory(ros_test)
