add_library(voronoi_graph crossing.cpp segment_expander.cpp segment.cpp thinning.cpp voronoi_graph.cpp)
target_link_libraries(voronoi_graph jsoncpp ${OpenCV_LIBRARIES})

# 参考资料