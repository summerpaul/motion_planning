/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-24 10:07:46
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-08-24 15:18:34
 */
#include <ros/ros.h>

#include <iostream>
using namespace std;
#include "gridmap_ros.h"
int main(int argc, char** argv) {
  ros::init(argc, argv, "gridmap_test");
  GridmapRos gridmap_ros;
  if (!gridmap_ros.init()) {
    std::cout << "grid map init failed " << std::endl;
  }
  ros::spin();
  return 0;
}
