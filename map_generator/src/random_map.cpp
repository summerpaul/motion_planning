/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-04 19:07:20
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-23 16:21:51
 */
#include "map_generator/random_map.h"

using namespace std;
RandomMap::RandomMap() : pnh_("~") {}
RandomMap::~RandomMap() {}
bool RandomMap::init() {
  std::string odom_topic;
  std::string local_map_topic;
  std::string all_map_topic;
  bool is_use_pcd_map;
  std::string pcd_map_path;

  // 初始位置
  pnh_.param("init_x", init_x_, 0.0);
  pnh_.param("init_y", init_y_, 0.0);
  pnh_.param("init_yaw", init_yaw_, 0.0);
  pnh_.param("map/x_size", x_size_, 50.0);
  pnh_.param("map/y_size", y_size_, 50.0);
  pnh_.param("map/obs_num", obs_num_, 30);
  pnh_.param("map/resolution", resolution_, 0.1);
  pnh_.param("ObstacleShape/lower_rad", w_l_, 0.3);
  pnh_.param("ObstacleShape/upper_rad", w_h_, 0.8);
  pnh_.param("sensing/radius", sensing_range_, 10.0);
  pnh_.param("is_use_pcd_map", is_use_pcd_map, false);
  pnh_.param<std::string>("pcd_map_path", pcd_map_path, "/");
  x_l_ = -x_size_ * 0.5;
  x_h_ = +x_size_ * 0.5;
  y_l_ = -y_size_ * 0.5;
  y_h_ = +y_size_ * 0.5;
  obs_num_ = min(obs_num_, (int)x_size_ * 10);
  if (!is_use_pcd_map) {
    randomMapGenerate();
  } else {
    if (!loadPcdMap(pcd_map_path)) {
      return false;
    }
  }

  std::cout << "generate random map " << std::endl;
  all_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);
  local_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  click_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);

  odom_sub_ = nh_.subscribe("odometry", 50, &RandomMap::odomCallback, this);
  rviz_odom_sub_ =
      nh_.subscribe("/initialpose", 1, &RandomMap::rvizPoseCallback, this);
  click_sub_ =
      nh_.subscribe("/clicked_point", 1, &RandomMap::clickCallback, this);
  senser_points_pub_timer_ =
      nh_.createTimer(ros::Duration(0.5), &RandomMap::pubSensedPoints, this);
  return true;
}

void RandomMap::clickCallback(const geometry_msgs::PointStamped &msg) {
  std::cout << "add click pcd" << std::endl;
  random_device rd;
  default_random_engine eng(rd());
  auto rand_w = uniform_real_distribution<double>(w_l_, w_h_);
  double x = msg.point.x;
  double y = msg.point.y;
  double w = rand_w(eng);
  pcl::PointXYZ pt_random;

  x = floor(x / resolution_) * resolution_ + resolution_ / 2.0;
  y = floor(y / resolution_) * resolution_ + resolution_ / 2.0;

  // 随机的宽度
  int widNum = ceil(w / resolution_);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      pt_random.x = x + (r + 0.5) * resolution_ + 1e-2;
      pt_random.y = y + (s + 0.5) * resolution_ + 1e-2;
      pt_random.z = 0;
      clicked_cloud_.points.push_back(pt_random);
      cloud_map_.points.push_back(pt_random);
    }
  cloud_map_.width = cloud_map_.points.size();
  kdtree_map_.setInputCloud(cloud_map_.makeShared());
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  return;
}
bool RandomMap::loadPcdMap(const std::string &file_path) {
  if (pcl::io::loadPCDFile(file_path, cloud_map_)) {
    std::cerr << "failed to open the input cloud" << std::endl;
    return false;
  }
  cloud_map_.width = cloud_map_.points.size();
  cloud_map_.height = 1;
  kdtree_map_.setInputCloud(cloud_map_.makeShared());
  map_ok_ = true;
  return true;
}
void RandomMap::randomMapGenerate() {
  random_device rd;
  default_random_engine eng(rd());
  auto rand_x = uniform_real_distribution<double>(x_l_, x_h_);
  auto rand_y = uniform_real_distribution<double>(y_l_, y_h_);
  auto rand_w = uniform_real_distribution<double>(w_l_, w_h_);
  pcl::PointXYZ pt_random;
  for (int i = 0; i < obs_num_; i++) {
    double x = rand_x(eng);
    double y = rand_y(eng);
    double w = rand_w(eng);
    // std::cout << "i is " << i << std::endl;

    if (sqrt(pow(x - init_x_, 2) + pow(y - init_y_, 2)) < 2.0) {
      i--;
      continue;
    }
    x = floor(x / resolution_) * resolution_ + resolution_ / 2;
    y = floor(y / resolution_) * resolution_ + resolution_ / 2;
    int widNum = ceil(w / resolution_);
    for (int r = -widNum / 2.0; r < widNum / 2.0; r++) {
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        pt_random.x = x + (r + 0.5) * resolution_ + 1e-2;
        pt_random.y = y + (s + 0.5) * resolution_ + 1e-2;
        pt_random.z = 0;
        cloud_map_.points.push_back(pt_random);
        // std::cout << "add random point " << std::endl;
      }
    }
  }
  cloud_map_.width = cloud_map_.points.size();
  cloud_map_.height = 1;
  kdtree_map_.setInputCloud(cloud_map_.makeShared());
  map_ok_ = true;
}

void RandomMap::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {
  has_odom_ = true;
  state_ = {odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z,
            odom->twist.twist.linear.x,
            odom->twist.twist.linear.y,
            odom->twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}
void RandomMap::rvizPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  has_odom_ = true;
  state_ = {msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0};
  std::cout << "receive rviz pose" << std::endl;
}

void RandomMap::pubSensedPoints(const ros::TimerEvent &) {
  if (cloud_map_.points.size() == 0) {
    std::cout << "random map not init" << std::endl;
    return;
  }
  pcl::toROSMsg(cloud_map_, global_map_pcd_);
  global_map_pcd_.header.frame_id = "map";
  all_map_pub_.publish(global_map_pcd_);
  // std::cout << "point size is " << cloud_map_.points.size() << std::endl;

  if (!map_ok_ || !has_odom_) {
    // std::cout << "map_ok_ is " << map_ok_ << " has_odom_ is " << has_odom_ <<
    // std::endl;
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> localMap;
  pcl::PointXYZ searchPoint(state_[0], state_[1], state_[2]);
  point_idx_radius_search_.clear();
  point_radius_squared_distance_.clear();
  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;
  if (kdtree_map_.radiusSearch(searchPoint, sensing_range_,
                               point_idx_radius_search_,
                               point_radius_squared_distance_) > 0) {
    for (size_t i = 0; i < point_idx_radius_search_.size(); ++i) {
      pt = cloud_map_.points[point_idx_radius_search_[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }
  // for (auto point : clicked_cloud_) {
  //   localMap.points.push_back(point);
  // }
  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;
  pcl::toROSMsg(localMap, local_map_pcd_);
  local_map_pcd_.header.frame_id = "map";
  local_map_pub_.publish(local_map_pcd_);
  pcl::toROSMsg(clicked_cloud_, local_map_pcd_);
  local_map_pcd_.header.frame_id = "map";
  click_map_pub_.publish(local_map_pcd_);
}
