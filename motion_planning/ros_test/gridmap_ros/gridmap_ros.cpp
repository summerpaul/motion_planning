/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-24 10:07:35
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-23 16:39:27
 */

#include "gridmap_ros.h"
GridmapRos::GridmapRos() : pnh_("~") {}
GridmapRos::~GridmapRos() {}
bool GridmapRos::init() {
  std::string local_pcd_map_topic_name;
  std::string global_pcd_map_topic_name;
  std::string local_occupancy_grid_map_topic_name;
  std::string global_occupancy_grid_map_topic_name;
  pnh_.param<std::string>("local_pcd_map_topic_name", local_pcd_map_topic_name,
                          "/map_generator/local_cloud");
  pnh_.param<std::string>("global_pcd_map_topic_name",
                          global_pcd_map_topic_name,
                          "/map_generator/global_cloud");
  pnh_.param<std::string>("local_occupancy_grid_map_topic_name",
                          local_occupancy_grid_map_topic_name,
                          "/local_occupancy_grid_map");
  pnh_.param<std::string>("global_occupancy_grid_map_topic_name",
                          global_occupancy_grid_map_topic_name,
                          "/global_occupancy_grid_map");
  local_pcd_map_sub_ = nh_.subscribe(local_pcd_map_topic_name, 50,
                                     &GridmapRos::localPcdMapCallback, this);
  global_pcd_map_sub_ = nh_.subscribe(global_pcd_map_topic_name, 50,
                                      &GridmapRos::globalPcdMapCallback, this);
  local_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      local_occupancy_grid_map_topic_name, 1);
  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);
  return true;
}
void GridmapRos::localPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  gridmap_.createGridmap(cloud, 0.05, 0.5);
  auto msg = gridmaptoRosMessage(gridmap_);
  local_occupancy_grid_map_pub_.publish(msg);
}
void GridmapRos::globalPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  gridmap_.createGridmap(cloud, 0.05, 0.5);
  auto msg = gridmaptoRosMessage(gridmap_);
  global_occupancy_grid_map_pub_.publish(msg);
}
nav_msgs::OccupancyGrid GridmapRos::gridmaptoRosMessage(
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
