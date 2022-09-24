add_executable(astar_search_grid_map_ros_node  astar_search_grid_map_ros_node.cpp astar_search_grid_map_ros.cpp)
target_link_libraries(astar_search_grid_map_ros_node
    global_planner
    ${catkin_LIBRARIES}
)
