/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 13:55:51
 */
#include <iostream>

using namespace std;
#include <ros/ros.h>

#include <iostream>
#include "global_planner_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner_test");
    GlobalPlannerRos global_planner;
    if(!global_planner.init()){
      std::cout << "fail to init global planner " << std::endl;
      return 1;
    }
    ros::spin();
    return 0;
}
