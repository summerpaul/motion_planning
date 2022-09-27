/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-24 10:07:35
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 08:55:18
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

  pnh_.param<bool>("save_global_map", save_global_map_, false);
  pnh_.param<std::string>("map_path", map_path_, "");
  local_pcd_map_sub_ = nh_.subscribe(local_pcd_map_topic_name, 50,
                                     &GridmapRos::localPcdMapCallback, this);
  global_pcd_map_sub_ = nh_.subscribe(global_pcd_map_topic_name, 50,
                                      &GridmapRos::globalPcdMapCallback, this);
  local_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      local_occupancy_grid_map_topic_name, 1);
  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);

  click_pose_sub_ =
      nh_.subscribe("/initialpose", 50, &GridmapRos::rvizPoseCallback, this);
  return true;
}
void GridmapRos::localPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  if (!local_gridmap_ptr_) {
    local_gridmap_ptr_ = std::make_shared<GridMap>();
  }
  local_gridmap_ptr_->createGridMap(cloud, 0.05, 0.5);
  auto msg = gridmaptoRosMessage(*local_gridmap_ptr_);
  local_occupancy_grid_map_pub_.publish(msg);
}
void GridmapRos::globalPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  if (!global_gridmap_ptr_) {
    global_gridmap_ptr_ = std::make_shared<GridMap>();
  }
  static bool is_create_grid_map{false};
  if (!is_create_grid_map) {
    global_gridmap_ptr_->createGridMap(cloud);
    is_create_grid_map = true;
  }
  auto msg = gridmaptoRosMessage(*global_gridmap_ptr_);
  global_occupancy_grid_map_pub_.publish(msg);
  static bool saved_map = false;
  if (save_global_map_ && !saved_map) {
    auto map = global_gridmap_ptr_->toImage();
    std::string map_path = map_path_ + "/map.jpg";
    cv::imwrite(map_path, map);
    saved_map = true;
    std::ofstream ofs(map_path_ + "/map.yaml");
    ofs << "image: map.png" << std::endl;
    ofs << "resolution: " << global_gridmap_ptr_->getResolution() << std::endl;
    ofs << "origin: [" << global_gridmap_ptr_->getRootX() << ", "
        << global_gridmap_ptr_->getRootY() << ", "
        << global_gridmap_ptr_->getRootTheta() << "]" << std::endl;
    ofs << "occupied_thresh: 0.5" << std::endl;
    ofs << "free_thresh: 0.2" << std::endl;
    ofs << "negate: 0" << std::endl;
    auto mat = global_gridmap_ptr_->toImage();
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE ); // Create a window for display.
    cv::imshow( "Display window", mat );                // Show our image inside it.
    cv::waitKey(0);
    
  }

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
void GridmapRos::rvizPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (!global_gridmap_ptr_) {
    std::cout << "global_gridmap_ptr_ is empty" << std::endl;
    return;
  }
  VehicleState pose;
  pose.position[0] = msg->pose.pose.position.x;
  pose.position[1] = msg->pose.pose.position.y;
  std::cout << "click pose is " << pose.position << std::endl;
  Eigen::Vector2i index = global_gridmap_ptr_->getGridMapIndex(pose.position);
  std::cout << "index is " << index << std::endl;
  std::cout << "index  Verify is" << global_gridmap_ptr_->isVerify(index)
            << std::endl;
  std::cout << "index  Occupied is" << global_gridmap_ptr_->isOccupied(index)
            << std::endl;
  Eigen::Vector2d pose_in_map = global_gridmap_ptr_->getCartesianCoordinate(index);
  std::cout << "pose_in_map is " << pose_in_map << std::endl;
}
