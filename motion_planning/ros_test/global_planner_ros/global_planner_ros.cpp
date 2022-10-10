/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-10 08:58:44
 */
/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 14:28:07
 */
#include "global_planner_ros.h"

GlobalPlannerRos::GlobalPlannerRos() : pnh_("~") {}
bool GlobalPlannerRos::init() {
  std::string global_pcd_map_topic_name;
  std::string global_path_topic_name;
  std::string planner_type;
  pnh_.param<std::string>("global_pcd_map_topic_name",
                          global_pcd_map_topic_name,
                          "/map_generator/global_cloud");
  pnh_.param<std::string>("planner_type", planner_type,
                          "astar_search_grid_map");

  global_pcd_map_sub_ =
      nh_.subscribe(global_pcd_map_topic_name, 50,
                    &GlobalPlannerRos::globalPcdMapCallback, this);
  start_pose_sub_ = nh_.subscribe(
      "/initialpose", 50, &GlobalPlannerRos::rvizStartPoseCallback, this);

  goal_pose_sub_ = nh_.subscribe("/move_base_simple/goal", 50,
                                 &GlobalPlannerRos::rvizGoalPoseCallback, this);
  global_gridmap_ptr_ = std::make_shared<GridMap>();

  plan_env_ptr_ = std::make_shared<PlanEnvrionment>();
  if (planner_type == "astar_search_grid_map") {
    global_planner_ptr_ = std::make_shared<AStarSearchGridMap>();
    std::cout << "create astar_search_grid_map " << std::endl;
  } else if (planner_type == "min_collision_risk_planner") {
    global_planner_ptr_ = std::make_shared<MinCollisionRiskPlanner>();
    std::cout << "create min_collision_risk_planner " << std::endl;
  } else if (planner_type == "kinodynamic_astar_grid_map") {
    global_planner_ptr_ = std::make_shared<KinodynamicAstarGridMap>();
    std::cout << "create kinodynamic_astar_grid_map " << std::endl;
  }

  else {
    return false;
  }

  return true;
}

void GlobalPlannerRos::globalPcdMapCallback(
    const sensor_msgs::PointCloud2::ConstPtr &pcd_map) {
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
  visualizer_.globalGridMapVis(global_gridmap_ptr_);

  if (!receive_glocal_map_flag_) {
    receive_glocal_map_flag_ = true;
    plan_env_ptr_->setGridMap(global_gridmap_ptr_);
    std::cout << "setGridMap "
              << "!global_gridmap_ptr_ is " << !global_gridmap_ptr_
              << std::endl;

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
void GlobalPlannerRos::rvizStartPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  start_pt_.position[0] = msg->pose.pose.position.x;
  start_pt_.position[1] = msg->pose.pose.position.y;
  start_pt_.theta = tf::getYaw(msg->pose.pose.orientation);

  has_start_pt_ = true;
}
void GlobalPlannerRos::rvizGoalPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  goal_pt_.position[0] = msg->pose.position.x;
  goal_pt_.position[1] = msg->pose.position.y;
  goal_pt_.theta = tf::getYaw(msg->pose.orientation);
  if (!has_start_pt_) {
    std::cout << "can not search, start node is empty" << std::endl;
    return;
  }
  global_planner_ptr_->reset();
  auto start_time = motion_planning::common::getTimeNow();
  auto flag = global_planner_ptr_->search(start_pt_, goal_pt_);
  if (flag == NO_PATH) {
    std::cout << "no path " << std::endl;
    return;
  }
  auto end_time = motion_planning::common::getTimeNow();
  std::cout << "dt is " << end_time - start_time << std::endl;
  std::cout << "get global path " << std::endl;
  auto global_search_path = global_planner_ptr_->getPath();
  std::cout << "global_search_path size is " << global_search_path.size()
            << std::endl;
  visualizer_.routeResultVis(global_search_path);

  auto way_points = global_planner_ptr_->getNodePath();
  visualizer_.wayPointsVis(way_points);
}
