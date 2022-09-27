/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 09:01:10
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 13:20:23
 */
#include <stdint.h>

#ifndef __ESDF_GAZEBO_ROS_H__
#define __ESDF_GAZEBO_ROS_H__
#include "plan_environment/esdf_map.h"
#include "plan_environment/grid_map.h"
#include <fstream>
#include <jsoncpp/json/json.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
using namespace motion_planning::plan_environment;
class ESDFGazeboRos {
public:
  ESDFGazeboRos();
  bool init();

private:
  void globalMapPubLoop(const ros::TimerEvent &);
  bool loadMapConfig(const std::string &map_config_path);
  nav_msgs::OccupancyGrid gridmaptoRosMessage(const GridMap &gridmap, const std::string &frame_id);
  nav_msgs::OccupancyGrid cvMapToRosMessage(const cv::Mat &mat);

  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

private:
  GridMap::Ptr global_gridmap_ptr_;
  GridMap::Ptr local_gridmap_ptr_;
  ESDFMap::Ptr local_esdf_map_ptr_;
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber laser_scan_sub_;
  ros::Publisher global_occupancy_grid_map_pub_;
  ros::Publisher local_occupancy_grid_map_pub_;
  ros::Publisher local_edsf_map_pub_;
  ros::Timer global_grid_map_pub_timer_;
  nav_msgs::OccupancyGrid global_occupancy_grid_map_;
  nav_msgs::OccupancyGrid local_occupancy_grid_map_;
  bool generate_grid_map_ = false;
  tf::TransformListener tfListener_;
  laser_geometry::LaserProjection projector_;
  std::string laser_scan_topic_;
  std::string laser_frame_id_;
  double robot_radius_;
};

#endif /* __ESDF_GAZEBO_ROS_H__ */
