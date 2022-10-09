/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-08 16:21:48
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 19:10:14
 */
#include <stdint.h>

#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__
#include "common/common.h"
#include "plan_environment/grid_map.h"
#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// ros的可视化类，做单一整理，方便后续使用
//
using namespace motion_planning::plan_environment;
using namespace motion_planning::common;
class Visualizer {
public:
  Visualizer() {
    local_occupancy_grid_map_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("local_grid_map", 1);
    global_occupancy_grid_map_pub_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("global_local_map", 1);
    obstacle_pub_ = nh_.advertise<visualization_msgs::Marker>("obstacles", 1);
    route_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("route_result", 1);
    way_points_pub_ =
        nh_.advertise<visualization_msgs::Marker>("way_points", 1);
  }

  // 发布全局栅格地图
  void globalGridMapVis(const GridMap::Ptr &gird_map,
                        const std::string &frame_id = "map") {
    global_occupancy_grid_map_pub_.publish(
        gridmaptoRosMessage(*gird_map, frame_id));
  }
  //   发布局部栅格地图
  void localGridMapVis(const GridMap::Ptr &gird_map,
                       const std::string &frame_id = "map") {
    local_occupancy_grid_map_pub_.publish(
        gridmaptoRosMessage(*gird_map, frame_id));
  }

  // 发布障碍物
  void obstaclesVis(const std::vector<Eigen::Vector2d> &ob_list,
                    const std::string &frame_id = "map") {
    visualization_msgs::Marker obstacles;
    obstacles.type = visualization_msgs::Marker::CUBE_LIST;
    geometry_msgs::Point point;
    obstacles.header.frame_id = frame_id;
    obstacles.header.stamp = ros::Time::now();
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    obstacles.pose = pose;
    obstacles.id = 0;

    int ob_num = ob_list.size();
    obstacles.points.resize(ob_num);
    obstacles.colors.resize(ob_num);
    obstacles.ns = "obstacles";
    obstacles.scale.x = 0.2;
    obstacles.scale.y = 0.2;
    obstacles.scale.z = 0.2;
    for (size_t i = 0; i < ob_num; i++) {
      obstacles.points[i].x = ob_list[i].x();
      obstacles.points[i].y = ob_list[i].y();
      obstacles.points[i].z = 0;
      obstacles.colors[i].r = 1;
      obstacles.colors[i].g = 0;
      obstacles.colors[i].b = 0;
      obstacles.colors[i].a = 1;
    }
    obstacle_pub_.publish(obstacles);
  }

  void wayPointsVis(const Path &path, const std::string &frame_id = "map") {
    visualization_msgs::Marker way_points;
    way_points.type = visualization_msgs::Marker::SPHERE_LIST;
    geometry_msgs::Point point;
    way_points.header.frame_id = frame_id;
    way_points.header.stamp = ros::Time::now();
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    // way_points.pose = pose;

    int ob_num = path.size();
    way_points.points.resize(ob_num);
    way_points.colors.resize(ob_num);
    way_points.ns = "obstacles";
    way_points.scale.x = 0.2;
    way_points.scale.y = 0.2;
    way_points.scale.z = 0.2;
    for (size_t i = 0; i < ob_num; i++) {
      way_points.points[i].x = path[i].position.x();
      way_points.points[i].y = path[i].position.y();
      way_points.points[i].z = 0;
      way_points.colors[i].r = 0;
      way_points.colors[i].g = 0;
      way_points.colors[i].b = 1;
      way_points.colors[i].a = 1;
    }
    way_points_pub_.publish(way_points);
  }
  void routeResultVis(const Path &path, const std::string &frame_id = "map") {
    visualization_msgs::MarkerArray route_result;
    visualization_msgs::Marker points_vis, path_vis;
    points_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    path_vis.type = visualization_msgs::Marker::LINE_STRIP;
    points_vis.header.frame_id = frame_id;
    points_vis.header.stamp = ros::Time::now();
    points_vis.ns = "points";
    path_vis.header.frame_id = frame_id;
    path_vis.header.stamp = ros::Time::now();
    path_vis.ns = "path";
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    points_vis.pose = pose;
    path_vis.pose = pose;
    points_vis.scale.x = 0.1;
    points_vis.scale.y = 0.1;
    points_vis.scale.z = 0.1;
    path_vis.scale.x = 0.01;
    path_vis.scale.y = 0.01;
    path_vis.scale.z = 0.01;
    int point_num = path.size();
    points_vis.points.resize(point_num);
    points_vis.colors.resize(point_num);
    path_vis.points.resize(point_num);
    path_vis.colors.resize(point_num);
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color_red, color_green;
    color_red.r = 1;
    color_red.a = 1;
    color_green.g = 1;
    color_green.a = 1;
    for (size_t i = 0; i < point_num; i++) {
      point.x = path[i].position.x();
      point.y = path[i].position.y();
      point.z = 0;
      path_vis.points[i] = point;
      points_vis.points[i] = point;
      path_vis.colors[i] = color_green;
      points_vis.colors[i] = color_red;
    }
    route_result.markers.push_back(points_vis);
    route_result.markers.push_back(path_vis);

    route_pub_.publish(route_result);
  }

private:
  nav_msgs::OccupancyGrid
  gridmaptoRosMessage(const GridMap &gridmap,
                      const std::string &frame_id = "map") {
    nav_msgs::OccupancyGrid occupancy_grid;
    geometry_msgs::Pose pose;
    pose.position.x = gridmap.getRootX();
    pose.position.y = gridmap.getRootY();
    pose.orientation.z = sin(gridmap.getRootTheta() * 0.5);
    pose.orientation.w = cos(gridmap.getRootTheta() * 0.5);
    occupancy_grid.info.map_load_time = ros::Time::now();
    occupancy_grid.header.frame_id = frame_id;
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

private:
  ros::NodeHandle nh_;
  ros::Publisher route_pub_;                     //全局规划可视化发布
  ros::Publisher way_points_pub_;                // 关键节点可视化发布
  ros::Publisher global_occupancy_grid_map_pub_; //全局栅格地图发布
  ros::Publisher local_occupancy_grid_map_pub_;  //局部栅格地图发布
  ros::Publisher local_esdf_map_pub_;            //局部esdf地图发布
  ros::Publisher trajectory_pub_;                //规划的轨迹发布
  ros::Publisher obstacle_pub_; //通过gridmap praycost得到的障碍物
};

#endif /* __VISUALIZER_H__ */
