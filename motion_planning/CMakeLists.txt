cmake_minimum_required(VERSION 3.0.2)
project(motion_planning_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


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
# include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(core)
