/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 15:20:14
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 16:01:55
 */
#include <stdint.h>

#ifndef __DWA_LOCAL_PLANNER_H__
#define __DWA_LOCAL_PLANNER_H__
#include "local_planner_interface.h"
namespace motion_planning {
namespace local_planner {

class DWALocalPlanner : public LocalPlannerInterface {
public:
  virtual bool plan(const PlanEnvrionment::Ptr &plan_env,
                    const VehicleState &current_pose, const Path &path,
                    Trajectory &traj) override;

private:
  void motion(VehicleState &state, const double &velocity, const double yawrate,
              const double &dt);
  double calc_to_goal_cost(const Trajectory &traj, const VehicleState &goal);
  double calc_speed_cost(const Trajectory &traj, const double &target_speed);

  double calc_obstacle_cost(const Trajectory &traj,
                            const ESDFMap::Ptr &esdf_ptr);
};
} // namespace local_planner
} // namespace motion_planning

#endif /* __DWA_LOCAL_PLANNER_H__ */
