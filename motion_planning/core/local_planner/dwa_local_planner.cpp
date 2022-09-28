/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 15:20:33
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 16:16:03
 */
#include <iostream>

#include "dwa_local_planner.h"

namespace motion_planning {
namespace local_planner {
bool DWALocalPlanner::plan(const PlanEnvrionment::Ptr &plan_env,
                           const VehicleState &current_pose, const Path &path,
                           Trajectory &traj) {
  if (!plan_env) {
    std::cout << "plan_env is null" << std::endl;
    return false;
  }
  auto esdf_map = plan_env->getESDFMap();
  if (!esdf_map) {
    std::cout << "grid map is null " << std::endl;
    return false;
  }
  double min_cost = 1e6;
  double min_obs_cost = min_cost;
  double min_goal_cost = min_cost;
  double min_speed_cost = min_cost;
  traj.clear();
  std::vector<Trajectory> trajectories;
  Trajectory best_traj;
  for (auto v = cfg_.dwa_robot_cfg.min_velocity;
       v < cfg_.dwa_robot_cfg.max_velocity;
       v += cfg_.dwa_config.velocity_resolution) {
    for (auto y = -cfg_.dwa_robot_cfg.max_yawrate;
         y < cfg_.dwa_robot_cfg.max_yawrate;
         y += cfg_.dwa_config.yawrate_resolution) {
      StampedVehicleState state;
      state.time = 0;
      state.pose = current_pose;
      state.pose.position = Eigen::Vector2d::Zero();
      state.pose.theta = 0;
      Trajectory traj;
      //   对加速度与角速度进行采样
      for (auto t = 0; t < cfg_.dwa_config.predict_time;
           t += cfg_.dwa_config.dt) {
        motion(state.pose, v, y, cfg_.dwa_config.dt);
        state.time += cfg_.dwa_config.dt;
        traj.push_back(state);
      }
      trajectories.push_back(traj);

      double to_goal_cost = calc_to_goal_cost(traj, path.back());
      double speed_cost =
          calc_speed_cost(traj, cfg_.dwa_robot_cfg.max_velocity);
      double obstacle_cost = calc_obstacle_cost(traj, esdf_map);
      double final_cost = cfg_.dwa_config.to_goal_cost_gain * to_goal_cost +
                          cfg_.dwa_config.speed_cost_gain * speed_cost +
                          cfg_.dwa_config.obstacle_cost_gain * obstacle_cost;
      if (min_cost >= final_cost) {
        min_goal_cost = cfg_.dwa_config.to_goal_cost_gain * to_goal_cost;
        min_obs_cost = cfg_.dwa_config.obstacle_cost_gain * obstacle_cost;
        min_speed_cost = cfg_.dwa_config.speed_cost_gain * speed_cost;
        min_cost = final_cost;
        best_traj = traj;
      }
    }
  }
  if (min_cost == 1e6) {
    return false;
  }
  traj = best_traj;
  return true;
}
void DWALocalPlanner::motion(VehicleState &state, const double &velocity,
                             const double yawrate, const double &dt) {
  state.theta += yawrate * cfg_.dwa_config.dt;
  state.position[0] += velocity * std::cos(state.theta) * dt;
  state.position[1] += velocity * std::sin(state.theta) * dt;
  state.speed[0] = velocity;
  state.angular_speed = yawrate;
}

double DWALocalPlanner::calc_to_goal_cost(const Trajectory &traj,
                                          const VehicleState &goal) {
  return (traj.back().pose.position - goal.position).norm();
}
double DWALocalPlanner::calc_speed_cost(const Trajectory &traj,
                                        const double &target_speed) {
  return fabs(target_speed - traj.back().pose.speed[0]);
}

// 都是局部的
double DWALocalPlanner::calc_obstacle_cost(const Trajectory &traj,
                                           const ESDFMap::Ptr &esdf_ptr) {
  double cost = 0.0;
  double min_dist = 1e3;
  for (const auto &state : traj) {
    double dist = esdf_ptr->getDistance(state.pose.position);
    if (dist <= esdf_ptr->getResolution()) {
      cost = 1e6;
      return cost;
    }
    min_dist = std::min(min_dist, dist);
  }
  cost = 1.0 / min_dist;
  return cost;
}
} // namespace local_planner
} // namespace motion_planning
