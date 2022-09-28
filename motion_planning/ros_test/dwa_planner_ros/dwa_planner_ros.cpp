/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 16:17:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 18:24:38
 */
#include "dwa_planner_ros.h"
#include <iostream>
using namespace std;
DWALocalPLannerRos::DWALocalPLannerRos() : pnh_("~") {}
bool DWALocalPLannerRos::init() {
  std::string map_config_path;
  pnh_.param<std::string>("map_config_path", map_config_path, "none");
  pnh_.param<double>("robot_radius", robot_radius_, 0.1);
  if (!loadMapConfig(map_config_path)) {
    return false;
  }
  std::string local_esdf_map_topic_name;
  std::string global_occupancy_grid_map_topic_name;
  std::string local_occupancy_grid_map_topic_name;
  std::string odom_topic;
  pnh_.param<std::string>("local_edf_map_topic_name", local_esdf_map_topic_name,
                          "/local_esdf_map");
  pnh_.param<std::string>("global_occupancy_grid_map_topic_name",
                          global_occupancy_grid_map_topic_name,
                          "/global_occupancy_grid_map");
  pnh_.param<std::string>("local_occupancy_grid_map_topic_name",
                          local_occupancy_grid_map_topic_name,
                          "/local_occupancy_grid_map");
  pnh_.param<std::string>("laser_scan_topic", laser_scan_topic_, "/laser_scan");
  pnh_.param<std::string>("laser_frame_id", laser_frame_id_, "/laser");

  pnh_.param<std::string>("odom_topic", odom_topic, "/odom");

  laser_scan_sub_ = nh_.subscribe(laser_scan_topic_, 1,
                                  &DWALocalPLannerRos::laserScanCallback, this);

  odom_sub_ =
      nh_.subscribe(odom_topic, 1, &DWALocalPLannerRos::odomCallback, this);
  goal_pose_sub_ =
      nh_.subscribe("/move_base_simple/goal", 50,
                    &DWALocalPLannerRos::rvizGoalPoseCallback, this);

  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);
  local_edsf_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(local_esdf_map_topic_name, 1);

  local_occupancy_grid_map_pub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("local_gridmap", 1);
  vis_timer_ =
      nh_.createTimer(ros::Duration(0.5), &DWALocalPLannerRos::visLoop, this);
  plan_timer_ =
      nh_.createTimer(ros::Duration(0.05), &DWALocalPLannerRos::planLoop, this);
  global_gridmap_ptr_ = std::make_shared<GridMap>();
  local_gridmap_ptr_ = std::make_shared<GridMap>();
//   local_esdf_map_ptr_ = std::make_shared<ESDFMap>();
  global_planner_ptr_ = std::make_unique<AStarSearchGridMap>();
  local_planner_ptr_ = std::make_unique<DWALocalPlanner>();
  global_plan_env_ptr_ = std::make_shared<PlanEnvrionment>();
  local_plan_env_ptr_ = std::make_shared<PlanEnvrionment>();
  plan_state_ = PlanState::UNINIT;

  return true;
}

// 显示的循环
void DWALocalPLannerRos::visLoop(const ros::TimerEvent &) {
  global_occupancy_grid_map_pub_.publish(global_occupancy_grid_map_);
  local_edsf_map_pub_.publish(local_esdf_map_);
  local_occupancy_grid_map_pub_.publish(local_occupancy_grid_map_);
}
//   加载仿真地图
bool DWALocalPLannerRos::loadMapConfig(const std::string &map_config_path) {
  if (map_config_path == "none") {
    std::cout << "map_config_path is none " << std::endl;
    return false;
  }
  std::ifstream ifs(map_config_path.data());
  Json::Value json_map_config;
  Json::Reader reader;
  if (!reader.parse(ifs, json_map_config)) {
    std::cout << "failed to parse json map config " << std::endl;
    return false;
  }
  std::cout << "json_map_config is " << json_map_config << std::endl;
  auto image_path = json_map_config["image"].asCString();
  // 读取单通道的地图
  cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

  if (image.empty()) {
    std::cout << "image is empty " << std::endl;
    return false;
  }
  if (!global_gridmap_ptr_) {
    global_gridmap_ptr_ = std::make_shared<GridMap>();
  }
  double resolution = json_map_config["resolution"].asDouble();
  double root_x = json_map_config["root_x"].asDouble();
  double root_y = json_map_config["root_y"].asDouble();
  double root_theta = json_map_config["root_theta"].asDouble();

  global_gridmap_ptr_->createGridMap(image, root_x, root_y, root_theta,
                                     resolution, robot_radius_);
  std::cout << "robot_radius_ is " << robot_radius_ << std::endl;

  global_occupancy_grid_map_ = gridmaptoRosMessage(*global_gridmap_ptr_, "map");
  auto mat = global_gridmap_ptr_->toImage();

  cv::Mat _distField;
  global_gridmap_ptr_->computeDistanceField(mat, _distField);
  global_plan_env_ptr_->setGridMap(global_gridmap_ptr_);
  //   cv::imshow("_distField", _distField); // Show our image inside it.
  //   cv::waitKey(0);
  if (!global_map_ready_) {
    global_map_ready_ = true;
    std::cout << "global_map_ready_ ready " << std::endl;
  }
  generate_grid_map_ = true;
  return true;
}
//   接受局部点云
void DWALocalPLannerRos::laserScanCallback(
    const sensor_msgs::LaserScan::ConstPtr &msg) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header = msg->header;
  projector_.transformLaserScanToPointCloud(laser_frame_id_, *msg, cloud,
                                            tfListener_);
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
  if (!local_gridmap_ptr_) {
    local_gridmap_ptr_ = std::make_shared<GridMap>();
  }
  local_plan_env_ptr_->setESDFMap(local_gridmap_ptr_);
  local_gridmap_ptr_->createGridMap(pcl_cloud, 0.05, robot_radius_);
  local_esdf_map_ptr_ = local_plan_env_ptr_->getESDFMap();
  local_occupancy_grid_map_ =
      gridmaptoRosMessage(*local_gridmap_ptr_, laser_frame_id_);
  local_occupancy_grid_map_pub_.publish(local_occupancy_grid_map_);
  pcl::PointCloud<pcl::PointXYZI> cloud_i;
  local_esdf_map_ptr_->getESDFPointCloud(cloud_i, laser_frame_id_);
  pcl::toROSMsg(cloud_i, local_esdf_map_);
  
  if (!local_map_ready_) {
    local_map_ready_ = true;
    std::cout << "local_map_ready_ " << std::endl;
  }
}
//   订阅仿真的车辆位置信息
void DWALocalPLannerRos::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  current_state_.position[0] = msg->pose.pose.position.x;
  current_state_.position[1] = msg->pose.pose.position.y;
  current_state_.theta = tf::getYaw(msg->pose.pose.orientation);
  current_state_.speed[0] = msg->twist.twist.linear.x;
  current_state_.speed[1] = msg->twist.twist.linear.y;
  current_state_.angular_speed = msg->twist.twist.angular.z;
  if (!odom_ready_) {
    odom_ready_ = true;
    std::cout << "odom_ready_ " << std::endl;
  }
}
//   订阅目标点
void DWALocalPLannerRos::rvizGoalPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal_state_.position[0] = msg->pose.position.x;
  goal_state_.position[1] = msg->pose.position.y;
  goal_state_.theta = tf::getYaw(msg->pose.orientation);
  if (!goal_ready_) {
    goal_ready_ = true;
    std::cout << "goal_ready_ " << std::endl;
  }
}

nav_msgs::OccupancyGrid
DWALocalPLannerRos::gridmaptoRosMessage(const GridMap &gridmap,
                                        const std::string &frame_id) {
  nav_msgs::OccupancyGrid occupancy_grid;
  geometry_msgs::Pose pose;
  pose.position.x = gridmap.getRootX();
  pose.position.y = gridmap.getRootY();
  //   std::cout << "gridmap.getRootX() is " << gridmap.getRootX() << std::endl;
  //   std::cout << "gridmap.getRootY() is " << gridmap.getRootY() << std::endl;
  pose.orientation.z = sin(gridmap.getRootTheta() * 0.5);
  pose.orientation.w = cos(gridmap.getRootTheta() * 0.5);
  occupancy_grid.info.map_load_time = ros::Time::now();
  occupancy_grid.header.frame_id = frame_id;
  occupancy_grid.header.stamp = ros::Time::now();
  occupancy_grid.data.resize(gridmap.getWidth() * gridmap.getHeight());
  for (int i = 0; i < gridmap.data().size(); i++) {
    occupancy_grid.data[i] = gridmap.data()[i];
  }
  occupancy_grid.info.resolution = gridmap.getResolution();
  occupancy_grid.info.width = gridmap.getWidth();
  occupancy_grid.info.height = gridmap.getHeight();
  occupancy_grid.info.origin = pose;
  return occupancy_grid;
}

//   控制的主循环
void DWALocalPLannerRos::planLoop(const ros::TimerEvent &) {

  bool ready =
      odom_ready_ && global_map_ready_ && local_map_ready_ && goal_ready_;

  static int count = 0;
  switch (plan_state_) {
  case PlanState::UNINIT: {
    count++;
    if (count == 50) {
      count = 0;
      std::cout << "UNINIT " << std::endl;
    }
    if (ready) {
      plan_state_ = PlanState::READY;
      std::cout << "change state to READY " << std::endl;
    }
    break;
  }

  case PlanState::READY: {
    std::cout << "planner is ready " << std::endl;
    plan_state_ = PlanState::GLOBAL_PLAN;
    std::cout << "change state to GLOBAL_PLAN " << std::endl;
    break;
  }
  case PlanState::GLOBAL_PLAN: {
    if (!global_planner_ptr_) {
      std::cout << "global_planner_ptr_ is null " << std::endl;
      plan_state_ = PlanState::STOP;
      std::cout << "change state to STOP " << std::endl;
    }
    std::cout << "in GLOBAL_PLAN" << std::endl;
    auto flag = global_planner_ptr_->search(current_state_, goal_state_);
    if (flag == NO_PATH) {
      plan_state_ = PlanState::STOP;
      break;
    }
    global_path_ = global_planner_ptr_->getPath();
    plan_state_ = PlanState::LOCAL_PLAN;
    std::cout << "change state to LOCAL_PLAN " << std::endl;
    break;
  }
  case PlanState::LOCAL_PLAN: {
    std::cout << "hahaha" << std::endl;
    break;
  }

  case PlanState::STOP: {
    count++;
    if (count == 50) {
      count = 0;
      std::cout << "STOP " << std::endl;
    }

    break;
  }
  }
}