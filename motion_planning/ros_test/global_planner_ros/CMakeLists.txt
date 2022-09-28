add_executable(global_planner_ros_node  global_planner_ros_node.cpp global_planner_ros.cpp)
target_link_libraries(global_planner_ros_node
    global_planner
    ${catkin_LIBRARIES}
)
