add_executable(dwa_planner_ros_node  dwa_planner_ros_node.cpp dwa_planner_ros.cpp)
target_link_libraries(dwa_planner_ros_node
    global_planner
    local_planner
    jsoncpp
    ${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)
