#ifndef ESDF_MAP_ROS_H_
#define ESDF_MAP_ROS_H_
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>

#include "common/common.h"
#include "plan_environment/esdf_map.h"
#include "plan_environment/grid_map.h"
#include "plan_environment/plan_environment.h"
using namespace motion_planning::plan_environment;
using namespace motion_planning::common;
class ESDFMapRos {
 public:
  ESDFMapRos();
  bool init();

 private:
  nav_msgs::OccupancyGrid gridmaptoRosMessage(const GridMap& gridmap);
  void localPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr& pcd_map);
  void globalPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr& pcd_map);

  void rvizPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);

 private:
  GridMap::Ptr global_gridmap_ptr_;
  GridMap::Ptr local_gridmap_ptr_;
  ESDFMap::Ptr local_esdf_map_ptr_;

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber local_pcd_map_sub_;
  ros::Subscriber global_pcd_map_sub_;
  ros::Publisher local_edsf_map_pub_;
  ros::Publisher global_occupancy_grid_map_pub_;
  ros::Subscriber click_pose_sub_;
  nav_msgs::OccupancyGrid global_occupancy_grid_map_;
};

#endif  // ESDF_MAP_ROS_H_