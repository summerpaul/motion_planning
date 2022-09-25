#ifndef POLYNOMIAL_TRAJ_ROS_H_
#define POLYNOMIAL_TRAJ_ROS_H_
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>
#include <vector>

#include "common/polynomial_traj.h"
#include "ros_viz_tools/ros_viz_tools.h"
using namespace motion_planning::common;
using ros_viz_tools::ColorMap;
using ros_viz_tools::RosVizTools;
class PolynomialTrajRos {
 public:
  PolynomialTrajRos();
  bool init();

 private:
  void polyTrajPathPublish(const std::vector<Eigen::Vector2d>& path);
  void rvizVehiclePoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

 private:
  PolynomialTraj poly_traj_;
  ros::Subscriber click_point_sub_;
  std::vector<geometry_msgs::Pose> click_points_;
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  std::shared_ptr<RosVizTools> poly_traj_path_vis_;
  double max_vel_ = 1.0;
  double max_acc_ = 2.0;
};

#endif  // MACRO
