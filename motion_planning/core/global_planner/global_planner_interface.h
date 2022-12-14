/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 18:53:56
 */
#ifndef MOTION_PLANNING_GLOBAL_PLANNER_INTERFACE_H_
#define MOTION_PLANNING_GLOBAL_PLANNER_INTERFACE_H_
#include <vector>

#include "common/common.h"
#include "plan_environment/plan_environment.h"
namespace motion_planning {
namespace global_planner {
using namespace common;
using namespace plan_environment;
enum { REACH_END = 1, NO_PATH = 2 };
class GlobalPlannerInterface {
 public:
  virtual ~GlobalPlannerInterface() {}
  virtual int search(const VehicleState& start_pt, const VehicleState& end_pt) = 0;
  virtual Path getPath(const double& delta_t = 0.1) = 0;
  virtual Path getNodePath() = 0;
  virtual void reset() = 0;
  virtual void setPlanEnvrionment(const PlanEnvrionment::Ptr& plan_env) = 0;
  typedef std::unique_ptr<GlobalPlannerInterface> Ptr;
};
}  // namespace global_planner
}  // namespace motion_planning
#endif  // GLOBAL_PLANNER_INTERFACE_H_