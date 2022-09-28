/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 14:31:45
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 15:45:38
 */
#include <stdint.h>

#ifndef __LOCAL_PLANNER_CONFIG_H__
#define __LOCAL_PLANNER_CONFIG_H__
#include <Eigen/Core>
// 局部规划的参数设置
namespace motion_planning {
namespace local_planner {

struct DWAConfig {
  double target_speed = 0.8;
  double velocity_resolution = 0.1;
  double yawrate_resolution = 0.1;
  double predict_time = 5.0;
  double to_goal_cost_gain = 1.0;
  double speed_cost_gain = 5.0;
  double obstacle_cost_gain = 10;
  double angle_resolution = 0.0087;
  double max_dist = 10.0;
  double goal_threshold = 0.3;
  double turn_direction_threshould = 0.1;
  double dt = 0.05;
};

struct DWARobotConfig {
  double max_velocity = 1.0;
  double min_velocity = 0;
  double max_yawrate = 0.8;
  double max_d_yawrate = 5.0;
  double max_acceleration = 1.0;
};
struct LocalPlannerConfig {
  DWAConfig dwa_config;
  DWARobotConfig dwa_robot_cfg;
};
} // namespace local_planner
} // namespace motion_planning

#endif /* __LOCAL_PLANNER_CONFIG_H__ */
