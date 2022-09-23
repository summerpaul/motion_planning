/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-04 19:16:46
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-08-19 09:23:44
 */
#include <ros/ros.h>

#include <iostream>

#include "map_generator/random_map.h"
using namespace std;

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  RandomMap random_map;
  if(!random_map.init()){
    cout << "fail to init random map" << std::endl;
  }
  ros::spin();
  return 0;
}
