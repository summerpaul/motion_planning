#include "astar_search_grid_map_ros.h"

AStarSearchGridMapRos::AStarSearchGridMapRos() : pnh_("~") {}
bool AStarSearchGridMapRos::init() {
  std::string global_pcd_map_topic_name;
  std::string global_occupancy_grid_map_topic_name;
  std::string global_path_topic_name;
  pnh_.param<std::string>("global_pcd_map_topic_name",
                          global_pcd_map_topic_name,
                          "/map_generator/global_cloud");

  pnh_.param<std::string>("global_occupancy_grid_map_topic_name",
                          global_occupancy_grid_map_topic_name,
                          "/global_occupancy_grid_map");

  pnh_.param<std::string>("global_path_topic_name", global_path_topic_name,
                          "/global_path");

  global_pcd_map_sub_ =
      nh_.subscribe(global_pcd_map_topic_name, 50,
                    &AStarSearchGridMapRos::globalPcdMapCallback, this);
  start_pose_sub_ = nh_.subscribe(
      "/initialpose", 50, &AStarSearchGridMapRos::rvizStartPoseCallback, this);

  goal_pose_sub_ =
      nh_.subscribe("/move_base_simple/goal", 50,
                    &AStarSearchGridMapRos::rvizGoalPoseCallback, this);

  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);

  global_path_pub_ = nh_.advertise<nav_msgs::Path>(global_path_topic_name, 1);
  global_gridmap_ptr_ = std::make_shared<GridMap>();
  global_planner_ptr_ = std::make_shared<AStarSearchGridMap>();
  plan_env_ptr_ = std::make_shared<PlanEnvrionment>();

  return true;
}

void AStarSearchGridMapRos::globalPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcd_map) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pcd_map, pcl_pc);
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  static bool is_create_grid_map{false};

  if (!is_create_grid_map) {
    global_gridmap_ptr_->createGridMap(cloud);
    is_create_grid_map = true;
    std::cout << "createGridMap" << std::endl;
  }

  auto msg = gridmaptoRosMessage(*global_gridmap_ptr_);
  global_occupancy_grid_map_pub_.publish(msg);

  if (!receive_glocal_map_flag_) {
    receive_glocal_map_flag_ = true;
    plan_env_ptr_->setGridMap(global_gridmap_ptr_);

    global_planner_ptr_->setPlanEnvrionment(plan_env_ptr_);
    std::cout << "map width is " << global_gridmap_ptr_->getWidth()
              << std::endl;
    std::cout << "map height is " << global_gridmap_ptr_->getHeight()
              << std::endl;
    std::cout << "map resolution is " << global_gridmap_ptr_->getResolution()
              << std::endl;

    std::cout << "init global planner " << std::endl;
  }
}
nav_msgs::OccupancyGrid AStarSearchGridMapRos::gridmaptoRosMessage(
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

void AStarSearchGridMapRos::rvizStartPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  start_pt_.position[0] = msg->pose.pose.position.x;
  start_pt_.position[1] = msg->pose.pose.position.y;
  start_pt_.theta = tf::getYaw(msg->pose.pose.orientation);
  has_start_pt_ = true;
}
void AStarSearchGridMapRos::rvizGoalPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal_pt_.position[0] = msg->pose.position.x;
  goal_pt_.position[1] = msg->pose.position.y;
  goal_pt_.theta = tf::getYaw(msg->pose.orientation);
  if (!has_start_pt_) {
    std::cout << "can not search, start node is empty" << std::endl;
    return;
  }
  global_planner_ptr_->reset();
  auto flag = global_planner_ptr_->search(start_pt_, goal_pt_);
  if (flag == NO_PATH) {
    std::cout << "no path " << std::endl;
    return;
  }
  std::cout << "get global path " << std::endl;
  auto global_search_path = global_planner_ptr_->getPath();
  std::cout << "global_search_path size is " << global_search_path.size() << std::endl;
  globalPathVisPub(global_search_path);
}

void AStarSearchGridMapRos::globalPathVisPub(
    const std::vector<VehicleState>& global_path) {
  nav_msgs::Path global_path_vis;
  global_path_vis.header.frame_id = "map";
  global_path_vis.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped path_point;
  for (auto point : global_path) {
    path_point.pose.position.x = point.position[0];
    path_point.pose.position.y = point.position[1];
    path_point.pose.position.z = 0.2;
    global_path_vis.poses.push_back(path_point);
  }
  global_path_pub_.publish(global_path_vis);
}