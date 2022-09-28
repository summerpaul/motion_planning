/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 10:30:21
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 10:55:35
 */
#include <iostream>

using namespace std;

#include "voronoi_graph_ros.h"

int main(int argc, char **argv) {
  /* code */
  ros::init(argc, argv, "esdf_gazebo_test");
  VoronoiGraphRos voronoi_graph_ros;
  if (!voronoi_graph_ros.init()) {
    return 1;
  }
  ros::spin();

  return 0;
}
