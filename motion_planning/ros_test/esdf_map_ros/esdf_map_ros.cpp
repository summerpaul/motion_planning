/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-26 15:05:36
 */
#include "esdf_map_ros.h"

ESDFMapRos::ESDFMapRos() : pnh_("~") {}
bool ESDFMapRos::init() {
  std::string local_pcd_map_topic_name;
  std::string global_pcd_map_topic_name;
  std::string local_esdf_map_topic_name;
  std::string global_occupancy_grid_map_topic_name;
  pnh_.param<std::string>("local_pcd_map_topic_name", local_pcd_map_topic_name,
                          "/map_generator/local_cloud");
  pnh_.param<std::string>("global_pcd_map_topic_name",
                          global_pcd_map_topic_name,
                          "/map_generator/global_cloud");
  pnh_.param<std::string>("local_edf_map_topic_name", local_esdf_map_topic_name,
                          "/local_esdf_map");
  pnh_.param<std::string>("global_occupancy_grid_map_topic_name",
                          global_occupancy_grid_map_topic_name,
                          "/global_occupancy_grid_map");

  local_pcd_map_sub_ = nh_.subscribe(local_pcd_map_topic_name, 50,
                                     &ESDFMapRos::localPcdMapCallback, this);

  global_pcd_map_sub_ = nh_.subscribe(global_pcd_map_topic_name, 50,
                                      &ESDFMapRos::globalPcdMapCallback, this);

  local_edsf_map_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>(local_esdf_map_topic_name, 1);
  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);

  click_pose_sub_ =
      nh_.subscribe("/move_base_simple/goal", 50, &ESDFMapRos::rvizPoseCallback, this);
  local_esdf_map_ptr_ = std::make_shared<ESDFMap>();
  local_gridmap_ptr_ = std::make_shared<GridMap>();
  global_gridmap_ptr_ = std::make_shared<GridMap>();

  return true;
}

nav_msgs::OccupancyGrid ESDFMapRos::gridmaptoRosMessage(
    const GridMap& gridmap) {
  nav_msgs::OccupancyGrid occupancy_grid;
  geometry_msgs::Pose pose;
  pose.position.x = gridmap.getRootX();
  pose.position.y = gridmap.getRootY();
  pose.orientation.z = sin(gridmap.getRootTheta() * 0.5);
  pose.orientation.w = cos(gridmap.getRootTheta() * 0.5);
  occupancy_grid.info.map_load_time = ros::Time::now();
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.header.stamp = ros::Time::now();
  std::vector<int8_t> out_data;
  for (auto meta_data : gridmap.data()) {
    if (meta_data == 255) {
      out_data.push_back(100);
    } else {
      out_data.push_back(0);
    }
  }
  occupancy_grid.data = out_data;
  occupancy_grid.info.resolution = gridmap.getResolution();
  occupancy_grid.info.width = gridmap.getWidth();
  occupancy_grid.info.height = gridmap.getHeight();
  occupancy_grid.info.origin = pose;
  return occupancy_grid;
}
void ESDFMapRos::localPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  local_gridmap_ptr_->createGridMap(cloud, 0.05, 0.5);
  local_esdf_map_ptr_->updateGridmap(local_gridmap_ptr_);
  local_esdf_map_ptr_->updateESDF2d();
  pcl::PointCloud<pcl::PointXYZI> cloud_i;
  local_esdf_map_ptr_->getESDFPointCloud(cloud_i, "map");
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud_i, cloud_msg);
  local_edsf_map_pub_.publish(cloud_msg);
}
void ESDFMapRos::globalPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  static bool is_create_grid_map{false};
  if (!is_create_grid_map) {
    global_gridmap_ptr_->createGridMap(cloud);
    is_create_grid_map = true;
    global_occupancy_grid_map_ = gridmaptoRosMessage(*global_gridmap_ptr_);
  }
  global_occupancy_grid_map_pub_.publish(global_occupancy_grid_map_);
}

void ESDFMapRos::rvizPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  VehicleState pose;
  pose.position[0] = msg->pose.position.x;
  pose.position[1] =msg->pose.position.y;
  std::cout << "click pose is " << pose.position << std::endl;
  double dist = local_esdf_map_ptr_->getDistance(pose.position);
  std::cout << "dist is " << dist << std::endl;
  Eigen::Vector2d grad;
  local_esdf_map_ptr_->evaluateEDTWithGrad( pose.position,dist,grad  );
  std::cout << "dist is " << dist << " grad is " << grad << std::endl;
}