/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 16:17:52
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 18:20:11
 */
#include <stdint.h>

#ifndef __DWA_PLANNER_ROS_H__
#define __DWA_PLANNER_ROS_H__
#include "global_planner/astar_search_grid_map.h"
#include "local_planner/dwa_local_planner.h"
#include "plan_environment/esdf_map.h"
#include "plan_environment/grid_map.h"
#include <fstream>
#include <jsoncpp/json/json.h>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
using namespace motion_planning::plan_environment;
using namespace motion_planning::global_planner;
using namespace motion_planning::local_planner;
class DWALocalPLannerRos {

public:
  DWALocalPLannerRos();
  bool init();

private:
  // 显示的循环
  void visLoop(const ros::TimerEvent &);
  //   加载仿真地图
  bool loadMapConfig(const std::string &map_config_path);
  //   接受局部点云
  void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
  //   订阅仿真的车辆位置信息
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
  //   订阅目标点
  void rvizGoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  //   控制的主循环
  void planLoop(const ros::TimerEvent &);

  nav_msgs::OccupancyGrid gridmaptoRosMessage(const GridMap &gridmap,
                                              const std::string &frame_id);

private:
  enum PlanState { UNINIT, READY, GLOBAL_PLAN, LOCAL_PLAN ,STOP} plan_state_;
  GridMap::Ptr global_gridmap_ptr_;
  GridMap::Ptr local_gridmap_ptr_;
  ESDFMap::Ptr local_esdf_map_ptr_;
  PlanEnvrionment::Ptr local_plan_env_ptr_;
  PlanEnvrionment::Ptr global_plan_env_ptr_ ;
  
  GlobalPlannerInterface::Ptr global_planner_ptr_;
  LocalPlannerInterface::Ptr local_planner_ptr_;
  
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Publisher global_occupancy_grid_map_pub_;
  ros::Publisher local_occupancy_grid_map_pub_;
  ros::Publisher local_edsf_map_pub_;

  ros::Timer vis_timer_;
  ros::Timer plan_timer_;
  nav_msgs::OccupancyGrid global_occupancy_grid_map_;
  nav_msgs::OccupancyGrid local_occupancy_grid_map_;
  sensor_msgs::PointCloud2 local_esdf_map_;
  bool generate_grid_map_ = false;
  tf::TransformListener tfListener_;
  laser_geometry::LaserProjection projector_;
  std::string laser_scan_topic_;
  std::string laser_frame_id_;
  double robot_radius_;
  Path global_path_;
  Trajectory local_traj_;

  VehicleState current_state_;
  VehicleState goal_state_;

  bool odom_ready_ = false;
  bool global_map_ready_ = false;
  bool local_map_ready_ = false;
  bool goal_ready_ = false;
};
#endif /* __DWA_PLANNER_ROS_H__ */
