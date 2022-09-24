#include <iostream>

using namespace std;
#include <ros/ros.h>

#include <iostream>
#include "astar_search_grid_map_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_planner_test");
    AStarSearchGridMapRos global_planner;
    if(!global_planner.init()){
      std::cout << "fail to init global planner " << std::endl;
      return 1;
    }
    ros::spin();
    return 0;
}
