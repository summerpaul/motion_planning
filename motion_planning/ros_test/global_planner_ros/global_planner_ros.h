/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 18:45:32
 */
#ifndef ASTAR_SEARCH_GRID_MAP_ROS_H_
#define ASTAR_SEARCH_GRID_MAP_ROS_H_
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

#include <memory>

#include "common/time.h"
#include "global_planner/astar_search_grid_map.h"
#include "global_planner/kinodynamic_astar_grid_map.h"
#include "global_planner/min_collision_risk_planner.h"
#include "plan_environment/grid_map.h"
#include "plan_environment/plan_environment.h"
#include "ros_viz_tools/ros_viz_tools.h"
#include "visualizer.h"
using ros_viz_tools::ColorMap;
using ros_viz_tools::RosVizTools;
using namespace motion_planning::plan_environment;
using namespace motion_planning::global_planner;

class GlobalPlannerRos {
public:
  GlobalPlannerRos();
  bool init();

private:
  void globalPcdMapCallback(const sensor_msgs::PointCloud2::ConstPtr &pcd_map);
  void rvizStartPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void rvizGoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

private:
  GridMap::Ptr global_gridmap_ptr_;
  PlanEnvrionment::Ptr plan_env_ptr_;
  std::shared_ptr<GlobalPlannerInterface> global_planner_ptr_;
  Visualizer visualizer_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber global_pcd_map_sub_;
  ros::Subscriber start_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  bool receive_glocal_map_flag_{false};
  VehicleState start_pt_;
  VehicleState goal_pt_;
  bool has_start_pt_{false};
  bool has_goal_pt_{false};
};
#endif