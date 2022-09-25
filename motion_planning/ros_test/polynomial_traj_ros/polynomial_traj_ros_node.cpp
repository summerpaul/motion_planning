/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-29 14:25:13
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-08-29 14:25:47
 */
#include <iostream>
#include <ros/ros.h>
#include "polynomial_traj_ros.h"
using namespace std;


int main(int argc, char** argv)
{
    /* code */
  ros::init(argc, argv, "polynomial_traj_ros_node");
  PolynomialTrajRos poly_traj;
  if(!poly_traj.init()){
  }
  ros::spin();
    return 0;
}
