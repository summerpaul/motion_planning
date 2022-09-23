project(gridmap_ros)


add_executable(gridmap_ros_node gridmap_ros_node.cpp gridmap_ros.cpp)
target_link_libraries(gridmap_ros_node
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)
