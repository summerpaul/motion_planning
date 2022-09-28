/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 16:18:01
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 16:24:50
 */
#include <iostream>
#include <ros/ros.h>
using namespace std;

#include "dwa_planner_ros.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "global_planner_test");
  DWALocalPLannerRos dwa_local_planner;
  if (!dwa_local_planner.init()) {
    std::cout << "fail to init global planner " << std::endl;
    return 1;
  }
  ros::spin();
  return 0;
}
