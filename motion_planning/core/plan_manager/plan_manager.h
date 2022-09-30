/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-29 09:06:16
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-30 16:26:10
 */
#include <stdint.h>

#ifndef __PLAN_MANAGER_H__
#define __PLAN_MANAGER_H__
#include "global_planner/astar_search_grid_map.h"
#include "local_planner/dwa_local_planner.h"
#include "plan_environment/esdf_map.h"
#include "plan_environment/grid_map.h"
using namespace motion_planning::plan_environment;
using namespace motion_planning::global_planner;
using namespace motion_planning::local_planner;
namespace motion_planning {
namespace plan_manager {
// 总体规划的框架
// 规划的流程，
// 1.获得请求，使用A星进行全局规划
// 2.简化轨迹，使用三次样条曲线进行拟合
// 3.使用局部规划器，规划轨迹与运动。
// 难点，何时进行全局轨迹的重规划
class PlanManager {
private:
  enum PlanState { UNINIT, READY, GLOBAL_PLAN, LOCAL_PLAN, STOP } plan_state_;
  GridMap::Ptr global_gridmap_ptr_;//用于全局A星规划路径
  GridMap::Ptr local_gridmap_ptr_;
  ESDFMap::Ptr local_esdf_map_ptr_;
  PlanEnvrionment::Ptr local_plan_env_ptr_;
  PlanEnvrionment::Ptr global_plan_env_ptr_;

  GlobalPlannerInterface::Ptr global_planner_ptr_;
  LocalPlannerInterface::Ptr local_planner_ptr_;
};
} // namespace plan_manager
} // namespace motion_planning

#endif /* __PLAN_MANAGER_H__ */
