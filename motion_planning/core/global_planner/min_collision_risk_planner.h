/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-08 09:51:18
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 18:55:50
 */
#include <stdint.h>

#ifndef __MIN_COLLISION_RISK_PLANNER_H__
#define __MIN_COLLISION_RISK_PLANNER_H__
#include "astar_search_grid_map.h"
#include "common/spline.h"
#include "global_planner_interface.h"
#include "common/tools.h"
namespace motion_planning ::global_planner {
// 舜安全局规划算法
class MinCollisionRiskPlanner : public GlobalPlannerInterface {
public:
  MinCollisionRiskPlanner();
  virtual int search(const VehicleState &start_pt,
                     const VehicleState &end_pt) override;
  virtual Path getPath(const double &delta_t = 0.05) override;
  virtual Path getNodePath() override { return getPath(); }
  virtual void reset() override;

  virtual ~MinCollisionRiskPlanner() {}

  virtual void
  setPlanEnvrionment(const PlanEnvrionment::Ptr &plan_env) override;

private:
  // 获取关键点
  Path simplifyWithRDP(const Path &path, const double &epsilon);
  // 寻找路径中，最远点
  const std::pair<int, double> findMaximumDistance(const Path &path);

private:
  GlobalPlannerInterface::Ptr astart_search_ptr_; // A星算法作为搜索的前端
  double key_vertices_obtain_tolerance_ = 0.1;
  double sample_gap_ = 0.1;
};

} // namespace motion_planning::global_planner

#endif /* __MIN_COLLISION_RISK_PLANNER_H__ */
