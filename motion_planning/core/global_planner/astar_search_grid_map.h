/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-30 16:22:44
 */
#ifndef MOTION_PLANNING_ASTAR_SEARCH_GRID_MAP_H_
#define MOTION_PLANNING_ASTAR_SEARCH_GRID_MAP_H_
#include "global_planner_interface.h"
namespace motion_planning {
namespace global_planner {
class AStarSearchGridMap : public GlobalPlannerInterface {
 public:
  AStarSearchGridMap();
  virtual int search(const VehicleState& start_pt,
                     const VehicleState& end_pt) override;
  virtual std::vector<VehicleState> getPath(
      const double& delta_t = 0.1) override;
  virtual void reset() override;

  virtual ~AStarSearchGridMap() {}

  virtual void setPlanEnvrionment(
      const PlanEnvrionment::Ptr& plan_env) override;

 private:
  void retrievePath(NodePtr end_node);

 private:
  std::vector<Eigen::Vector2i> motions_;
  GridMap::Ptr grid_map_ptr_;
  VehicleState end_pt_;
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set_;
  NodeHashTable expanded_nodes_;
  std::vector<NodePtr> path_nodes_;
};
}  // namespace global_planner
}  // namespace motion_planning
#endif  // MOTION_PLANNING_ASTAR_SEARCH_GRID_MAP_H_