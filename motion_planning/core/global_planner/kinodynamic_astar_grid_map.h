/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-27 09:57:36
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-10 09:49:09
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

  virtual Path getNodePath() override {
    std::vector<VehicleState> path;
    for (int i = 0; i < path_nodes_.size(); ++i) {
      path.push_back(path_nodes_[i]->state);
      // std::cout << "path_nodes_" << i << "speed is \n" << path_nodes_[i]->state.speed << std::endl;
    }
    path.push_back(end_pt_);
    return path;
  }
  virtual void reset() override;

  virtual ~KinodynamicAstarGridMap() {}

  virtual void
  setPlanEnvrionment(const PlanEnvrionment::Ptr &plan_env) override;

private:
  double estimateHeuristic(const VehicleState &currt_pt,
                           const VehicleState &target_pt, double &optimal_time);
  std::vector<double> quartic(double a, double b, double c, double d, double e);
  std::vector<double> cubic(double a, double b, double c, double d);

  // 得到一条到终点的曲线
  bool computeShotTraj(const VehicleState &state1, const VehicleState &state2,
                       const double &time_to_goal);
  void stateTransit(VehicleState &state0, VehicleState &state1,
                    const Eigen::Vector2d &um, const double &tau);
  void retrievePath(NodePtr end_node);

private:
  GridMap::Ptr grid_map_ptr_;
  VehicleState end_pt_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;
  NodeHashTable expanded_nodes_;
  std::vector<NodePtr> path_nodes_;
  Eigen::Vector2d start_val_;
  Eigen::Vector2d start_acc_;
  bool is_shot_succ_{false};
  Eigen::Vector2d end_vel_;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  double max_vel_{1.5}, max_acc_{2.0}, max_tau_{0.5}, check_num_{10};
};
} // namespace global_planner
} // namespace motion_planning

#endif /* __KINODYNAMIC_ASTAR_GRID_MAP_H__ */
