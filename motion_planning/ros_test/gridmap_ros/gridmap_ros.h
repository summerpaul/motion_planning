/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-24 10:07:24
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-23 16:32:02
 */
#include <stdint.h>

#ifndef __GRIDMAP_ROS_H__
#define __GRIDMAP_ROS_H__
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "plan_environment/grid_map.h"
#include <fstream>
using namespace motion_planning::plan_environment;
class GridmapRos {
 public:
  GridmapRos();
  ~GridmapRos();
  bool init();

 private:
  nav_msgs::OccupancyGrid gridmaptoRosMessage(const GridMap& gridmap);
  void localPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr& pcd_map);
  void globalPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr& pcd_map);

 private:
  GridMap::Ptr global_gridmap_ptr_;
  GridMap::Ptr local_gridmap_ptr_;
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber local_pcd_map_sub_;
  ros::Subscriber global_pcd_map_sub_;
  ros::Publisher local_occupancy_grid_map_pub_;
  ros::Publisher global_occupancy_grid_map_pub_;
  bool save_global_map_ = false;
  std::string map_path_;
};

#endif /* __GRIDMAP_ROS_H__ */
