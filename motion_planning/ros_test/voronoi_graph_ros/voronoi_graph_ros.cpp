/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 10:30:16
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 13:28:30
 */
#include "voronoi_graph_ros.h"

VoronoiGraphRos::VoronoiGraphRos() : pnh_("~") {}
bool VoronoiGraphRos::init() {
  std::string map_config_path;
  pnh_.param<std::string>("map_config_path", map_config_path, "none");
  pnh_.param<double>("robot_radius", robot_radius_, 0.1);
  pnh_.param<float>("segment_length", segment_length_, 0.3);
  pubSegments_ = std::make_shared<RosVizTools>(nh_, "voronoi_path");
  if (!loadMapConfig(map_config_path)) {
    return false;
  }
  std::string global_occupancy_grid_map_topic_name;
  pnh_.param<std::string>("global_occupancy_grid_map_topic_name",
                          global_occupancy_grid_map_topic_name,
                          "/global_occupancy_grid_map");
  global_occupancy_grid_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(
      global_occupancy_grid_map_topic_name, 1);

  vis_timer_ =
      nh_.createTimer(ros::Duration(0.5), &VoronoiGraphRos::visLoop, this);
  return true;
}
void VoronoiGraphRos::visLoop(const ros::TimerEvent &) {
  pubSegments_->publish();
  global_occupancy_grid_map_pub_.publish(global_occupancy_grid_map_);
}
void VoronoiGraphRos::generateVoronoiGraphVis(const double &resolution,
                                              const Eigen::Vector2d &origin) {

  std::string ns, frame_id("map");
  std_msgs::ColorRGBA lane_color = ros_viz_tools::BLUE;
  geometry_msgs::Point p;
  Json::Value json_segs;
  voronoi_graph_.getJsonSegments(json_segs, segments_, origin, resolution);
  std::cout << "origin is " << origin << std::endl;
  std::cout << "json_segs size is " << json_segs.size() << std::endl;
  p.z = 0;
  pubSegments_->clear();
  for (auto segment : json_segs) {

    ns = "lane" + segment["id"].asString();
    visualization_msgs::Marker marker_linestrip =
        RosVizTools::newLineStrip(0.05, ns, 0, lane_color, frame_id);
    ns = "lane" + segment["id"].asString() + "points";
    visualization_msgs::Marker marker_nodelist =
        RosVizTools::newSphereList(0.2, ns, 0, ros_viz_tools::RED, frame_id);
    p.x = segment["start"]["x"].asDouble();
    p.y = segment["start"]["y"].asDouble();
    marker_nodelist.points.push_back(p);
    marker_linestrip.points.push_back(p);
    p.x = segment["end"]["x"].asDouble();
    p.y = segment["end"]["y"].asDouble();
    marker_nodelist.points.push_back(p);
    marker_linestrip.points.push_back(p);

    pubSegments_->append(marker_nodelist);
    pubSegments_->append(marker_linestrip);
  }
}
bool VoronoiGraphRos::loadMapConfig(const std::string &map_config_path) {
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
  global_occupancy_grid_map_ = gridmaptoRosMessage(*global_gridmap_ptr_, "map");

  auto mat = global_gridmap_ptr_->toImage();

  //   std::unique_ptr<float[]> potential;
  potential.reset(new float[mat.cols * mat.rows]);

  voronoi_graph_.calcSegments(
      segments_, mat, potential.get(), segment_length_ / resolution,
      crossingOptimization_ / resolution, endSegmentOptimization_ / resolution);
  std::cout << "segments_ size is " << segments_.size() << std::endl;
  generateVoronoiGraphVis(resolution, global_gridmap_ptr_->getOrigin());
  return true;
}

nav_msgs::OccupancyGrid
VoronoiGraphRos::gridmaptoRosMessage(const GridMap &gridmap,
                                     const std::string &frame_id) {
  nav_msgs::OccupancyGrid occupancy_grid;
  geometry_msgs::Pose pose;
  pose.position.x = gridmap.getRootX();
  pose.position.y = gridmap.getRootY();
  //   std::cout << "gridmap.getRootX() is " << gridmap.getRootX() <<
  //   std::endl; std::cout << "gridmap.getRootY() is " << gridmap.getRootY()
  //   << std::endl;
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