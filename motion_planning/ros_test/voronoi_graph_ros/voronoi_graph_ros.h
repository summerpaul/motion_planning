/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 10:30:12
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 13:30:47
 */
#include <stdint.h>

#ifndef __VORONOI_GRAPH_ROS_H__
#define __VORONOI_GRAPH_ROS_H__

#include "plan_environment/grid_map.h"
#include "plan_environment/voronoi_graph/voronoi_graph.h"
#include "ros_viz_tools/ros_viz_tools.h"
#include <nav_msgs/OccupancyGrid.h>
#include <jsoncpp/json/json.h>
using ros_viz_tools::ColorMap;
using ros_viz_tools::RosVizTools;
using namespace motion_planning::plan_environment;
class VoronoiGraphRos {
public:
  VoronoiGraphRos();
  bool init();

private:
  void generateVoronoiGraphVis(const double & resolution, const Eigen::Vector2d& origin);
  bool loadMapConfig(const std::string &map_config_path);
  void visLoop(const ros::TimerEvent &);
  nav_msgs::OccupancyGrid gridmaptoRosMessage(const GridMap &gridmap, const std::string &frame_id);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GridMap::Ptr global_gridmap_ptr_;
  VoronoiGraph voronoi_graph_;
  std::shared_ptr<RosVizTools> pubSegments_;
  ros::Publisher global_occupancy_grid_map_pub_;
  nav_msgs::OccupancyGrid global_occupancy_grid_map_;
  std::vector<Segment> segments_;
  float segment_length_ = 1.0;
  float crossingOptimization_ = 0.2;
  float endSegmentOptimization_ = 0.5;
  ros::Timer vis_timer_;
  double robot_radius_;
  std::unique_ptr<float[]> potential;
};

#endif /* __VORONOI_GRAPH_ROS_H__ */
