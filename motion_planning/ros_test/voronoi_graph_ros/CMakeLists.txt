add_executable(voronoi_graph_ros_node voronoi_graph_ros_node.cpp voronoi_graph_ros.cpp)
target_link_libraries(voronoi_graph_ros_node 
${PCL_LIBRARIES}
${OpenCV_LIBRARIES}
${catkin_LIBRARIES}
voronoi_graph
jsoncpp)