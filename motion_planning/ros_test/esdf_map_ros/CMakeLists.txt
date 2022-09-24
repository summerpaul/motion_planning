add_executable(esdf_map_ros_node esdf_map_ros.cpp esdf_map_ros_node.cpp)
target_link_libraries(esdf_map_ros_node
    ${catkin_LIBRARIES}
)
