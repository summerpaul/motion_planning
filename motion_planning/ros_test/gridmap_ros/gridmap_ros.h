/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-24 10:07:24
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 10:05:45
 */
#include <stdint.h>

#ifndef __GRIDMAP_ROS_H__
#define __GRIDMAP_ROS_H__
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

#include "common/common.h"
#include "plan_environment/grid_map.h"
#include "visualizer.h"
#include <visualization_msgs/MarkerArray.h>
using namespace motion_planning::plan_environment;
using namespace motion_planning::common;
class GridmapRos {
public:
  GridmapRos();
  ~GridmapRos();
  bool init();

private:
  void localPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr &pcd_map);
  void globalPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr &pcd_map);

  void rvizPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

private:
  GridMap::Ptr global_gridmap_ptr_;
  GridMap::Ptr local_gridmap_ptr_;

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber local_pcd_map_sub_;
  ros::Subscriber global_pcd_map_sub_;
  ros::Subscriber click_pose_sub_;
  bool save_global_map_ = false;
  std::string map_path_;
  Visualizer visualizer_;
};

#endif /* __GRIDMAP_ROS_H__ */
