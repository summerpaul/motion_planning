/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-08 09:51:33
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 10:47:35
 */
#include <iostream>

#include "min_collision_risk_planner.h"

namespace motion_planning ::global_planner {
MinCollisionRiskPlanner::MinCollisionRiskPlanner() {
  astart_search_ptr_ = std::make_unique<AStarSearchGridMap>();
}

int MinCollisionRiskPlanner::search(const VehicleState &start_pt,
                                    const VehicleState &end_pt) {
  auto flag = astart_search_ptr_->search(start_pt, end_pt);
  if (flag == NO_PATH) {
    return NO_PATH;
  }
  return REACH_END;
}
Path MinCollisionRiskPlanner::getPath(const double &delta_t) {

  Path key_vertices = simplifyWithRDP(astart_search_ptr_->getPath(),
                                      key_vertices_obtain_tolerance_);

  std::vector<double> path_x, path_y;
  for (auto point : key_vertices) {
    path_x.push_back(point.position.x());
    path_y.push_back(point.position.y());
  }
  Spline2D spline(path_x, path_y);

  // 对三次样条曲线进行采样
  auto samples =
      linspace(0.0, spline.s.back(), int(spline.s.back() / sample_gap_));
  Path out_path;
  VehicleState state;
  for (auto s : samples) {
    state.position = spline.calc_postion(s);
    state.kappa = spline.calc_curvature(s);
    state.theta = spline.calc_yaw(s);
    out_path.push_back(state);
  }
  return out_path;
}
void MinCollisionRiskPlanner::setPlanEnvrionment(
    const PlanEnvrionment::Ptr &plan_env) {
  astart_search_ptr_->setPlanEnvrionment(plan_env);
}

void MinCollisionRiskPlanner::reset() { astart_search_ptr_->reset(); }

Path MinCollisionRiskPlanner::simplifyWithRDP(const Path &path,
                                              const double &epsilon) {
  //  数据点少的时候，不进行简化

  if (path.size() < 3) {
    return path;
  }

  auto max_dist_pair = findMaximumDistance(path);
  if (max_dist_pair.second >= epsilon) {
    int index = max_dist_pair.first;
    auto it = path.begin();
    Path path1(path.begin(), it + index + 1);
    Path path2(it + index, path.end());
    Path r1 = simplifyWithRDP(path1, epsilon);
    Path r2 = simplifyWithRDP(path2, epsilon);
    Path key_vertices(r1);
    key_vertices.pop_back();
    key_vertices.insert(key_vertices.end(), r2.begin(), r2.end());
    return key_vertices;
  } else {
    // 说明是一条直线，只需要保留起点与终点
    Path key_vertices;
    key_vertices.push_back(path.front());
    key_vertices.push_back(path.back());
    return key_vertices;
  }
}
const std::pair<int, double>
MinCollisionRiskPlanner::findMaximumDistance(const Path &path) {
  Eigen::Vector2d first_point_pose = path.front().position;
  Eigen::Vector2d last_point_pose = path.back().position;
  int index = 0;
  double max_dist = -1;
  Eigen::Vector2d p = last_point_pose - first_point_pose;
  for (size_t i = 1; i < path.size() - 1; i++) {
    Eigen::Vector2d pp = path[i].position - first_point_pose;
    // 向量的正弦值
    double cosVal = p.dot(pp) / (p.norm() * pp.norm());
    double dist = pp.norm() * sin(acos(cosVal));
    if (dist > max_dist) {
      max_dist = dist;
      index = i;
    }
  }
  return std::make_pair(index, max_dist);
}
} // namespace motion_planning::global_planner