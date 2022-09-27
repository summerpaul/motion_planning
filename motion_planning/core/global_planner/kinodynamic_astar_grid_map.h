/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-27 09:57:36
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 17:08:23
 */
#include <stdint.h>

#ifndef __KINODYNAMIC_ASTAR_GRID_MAP_H__
#define __KINODYNAMIC_ASTAR_GRID_MAP_H__

#include "global_planner_interface.h"

namespace motion_planning {
namespace global_planner {
class KinodynamicAstarGridMap : public GlobalPlannerInterface {
public:
  KinodynamicAstarGridMap();
  virtual int search(const VehicleState &start_pt,
                     const VehicleState &end_pt) override;
  virtual std::vector<VehicleState>
  getPath(const double &delta_t = 0.1) override;
  virtual void reset() override;

  virtual ~KinodynamicAstarGridMap() {}

  virtual void
  setPlanEnvrionment(const PlanEnvrionment::Ptr &plan_env) override;

private:
  void estimateHeuristic(const VehicleState &currt_pt,
                         const VehicleState &target_pt, double &optical_time);
  vector<double> quartic(double a, double b, double c, double d, double e);
  vector<double> cubic(double a, double b, double c, double d);

private:
  GridMap::Ptr grid_map_ptr_;
  VehicleState end_pt_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;
  NodeHashTable expanded_nodes_;
  std::vector<NodePtr> path_nodes_;
  Eigen::Vector2d start_val_;
  Eigen::Vector2d start_acc_;
};
} // namespace global_planner
} // namespace motion_planning

#endif /* __KINODYNAMIC_ASTAR_GRID_MAP_H__ */
