/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-04 19:07:10
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-01 19:31:04
 */

#ifndef __RANDOM_MAP_H__
#define __RANDOM_MAP_H__
// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// ros
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// c++
#include <cmath>
#include <iostream>
#include <random>
#include <string>
// eigen
#include <Eigen/Eigen>

class RandomMap {
 public:
  RandomMap();
  ~RandomMap();
  bool init();

 private:
  void randomMapGenerate();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void rvizPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void pubSensedPoints(const ros::TimerEvent&);
  bool loadPcdMap(const std::string & file_path);
  void clickCallback(const geometry_msgs::PointStamped& msg);

 private:
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher local_map_pub_;
  ros::Publisher all_map_pub_;
  ros::Publisher click_map_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rviz_odom_sub_;
  ros::Subscriber click_sub_;
  ros::Timer senser_points_pub_timer_;
  

  // map_info
  int obs_num_;    //障碍物的数量
  double x_size_;  //
  double y_size_;
  double resolution_;

  double x_l_;
  double x_h_;
  double y_l_;
  double y_h_;
  double w_l_;
  double w_h_;
  // vehicle_init
  double init_x_;
  double init_y_;
  double init_yaw_;

  // sensor
  double sense_rate_;
  double sensing_range_;
  //
  bool map_ok_{false};
  bool has_odom_{false};
  pcl::PointCloud<pcl::PointXYZ> cloud_map_;
  pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;
  std::vector<double> state_;
  sensor_msgs::PointCloud2 global_map_pcd_;
  sensor_msgs::PointCloud2 local_map_pcd_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_map_;
  std::vector<int> point_idx_radius_search_;
  std::vector<float> point_radius_squared_distance_;
};

#endif /* __RANDOM_MAP_H__ */
