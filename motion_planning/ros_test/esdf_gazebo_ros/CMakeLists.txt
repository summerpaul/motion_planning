add_executable(esdf_gazebo_ros_node esdf_gazebo_ros_node.cpp esdf_gazebo_ros.cpp)
target_link_libraries(esdf_gazebo_ros_node 
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${catkin_LIBRARIES}
jsoncpp)