/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 09:01:14
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 14:02:40
 */
#include <iostream>

#include "esdf_gazebo_ros.h"

ESDFGazeboRos::ESDFGazeboRos() : pnh_("~") {}
bool ESDFGazeboRos::init() {
  std::string map_config_path;
  pnh_.param<std::string>("map_config_path", map_config_path, "none");
  pnh_.param<double>("robot_radius", robot_radius_, 0.1);
  if (!loadMapConfig(map_config_path)) {
    return false;
  }
  std::string local_esdf_map_topic_name;
  std::string global_occupancy_grid_map_topic_name;
  std::string local_occupancy_grid_map_topic_name;

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
  laser_scan_sub_ = nh_.subscribe(laser_scan_topic_, 1,
                                  &ESDFGazeboRos::laserScanCallback, this);
  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);
  local_edsf_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(local_esdf_map_topic_name, 1);

  local_occupancy_grid_map_pub_ =
      nh_.advertise<nav_msgs::OccupancyGrid>("local_gridmap", 1);
  global_grid_map_pub_timer_ = nh_.createTimer(
      ros::Duration(0.5), &ESDFGazeboRos::globalMapPubLoop, this);
  tfListener_.setExtrapolationLimit(ros::Duration(0.1));
  return true;
}

void ESDFGazeboRos::globalMapPubLoop(const ros::TimerEvent &) {
  if (!generate_grid_map_) {
    return;
  }
  //   std::cout << " publish map " << std::endl;
  global_occupancy_grid_map_pub_.publish(global_occupancy_grid_map_);
}

bool ESDFGazeboRos::loadMapConfig(const std::string &map_config_path) {

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
  cv::imshow("global_gridmap_ptr_", mat);    // Show our image inside it.
  cv::waitKey(0);
  std::cout << "mat is " <<  mat << std::endl;

  cv::Mat _distField;
  global_gridmap_ptr_->computeDistanceField(mat, _distField);
  cv::imshow("_distField", _distField); // Show our image inside it.
  cv::waitKey(0);

  generate_grid_map_ = true;
  return true;
}

nav_msgs::OccupancyGrid
ESDFGazeboRos::gridmaptoRosMessage(const GridMap &gridmap,
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

void ESDFGazeboRos::laserScanCallback(
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
  if (!local_esdf_map_ptr_) {
    local_esdf_map_ptr_ = std::make_shared<ESDFMap>();
  }
  local_gridmap_ptr_->createGridMap(pcl_cloud, 0.05, robot_radius_);
  local_esdf_map_ptr_->updateGridmap(local_gridmap_ptr_);
  local_esdf_map_ptr_->updateESDF2d();
  local_occupancy_grid_map_ =
      gridmaptoRosMessage(*local_gridmap_ptr_, laser_frame_id_);
  local_occupancy_grid_map_pub_.publish(local_occupancy_grid_map_);
  pcl::PointCloud<pcl::PointXYZI> cloud_i;
  local_esdf_map_ptr_->getESDFPointCloud(cloud_i, laser_frame_id_);
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_i, cloud_msg);
  local_edsf_map_pub_.publish(cloud_msg);
}
// test
nav_msgs::OccupancyGrid
ESDFGazeboRos::cvMapToRosMessage(const cv::Mat &cv_image) {
  nav_msgs::OccupancyGrid occupancy_grid;
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.orientation.z = sin(0);
  pose.orientation.w = cos(0);
  occupancy_grid.info.map_load_time = ros::Time::now();
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.header.stamp = ros::Time::now();
  std::vector<int8_t> out_data;

  int rows = cv_image.rows, cols = cv_image.cols;
  occupancy_grid.data.resize(rows * cols);
  for (int x = 0; x < rows; x++) {
    auto data = cv_image.ptr<uchar>(rows - 1 - x);
    for (int y = 0; y < cols; y++)
      occupancy_grid.data[y * rows + x] = 100 - *(data + cols - 1 - y) * 100 / 255;
  }

  occupancy_grid.info.resolution = 0.05;
  occupancy_grid.info.width = rows;
  occupancy_grid.info.height = cols;
  occupancy_grid.info.origin = pose;
  return occupancy_grid;
}