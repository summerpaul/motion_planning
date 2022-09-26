/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 09:01:20
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-26 10:05:59
 */
#include "esdf_gazebo_ros.h"


int main(int argc, char **argv) {
  /* code */
  ros::init(argc, argv, "esdf_gazebo_test");
  ESDFGazeboRos esdf_map;
  if (!esdf_map.init()) {
    return 1;
  }
  ros::spin();

  return 0;
}
