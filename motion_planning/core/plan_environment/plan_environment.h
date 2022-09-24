#ifndef MOTION_PLANNING_PLAN_ENVRIONMENT_H_
#define MOTION_PLANNING_PLAN_ENVRIONMENT_H_
#include "grid_map.h"
namespace motion_planning {
namespace plan_environment {
class PlanEnvrionment {
 public:
  void setGridMap(const GridMap::Ptr& grid_map) { grid_map_ptr_ = grid_map; }
  GridMap::Ptr getGridMap() const { return grid_map_ptr_; }

 private:
  GridMap::Ptr grid_map_ptr_;

 public:
  typedef std::shared_ptr<PlanEnvrionment> Ptr;
};
}  // namespace plan_environment
}  // namespace motion_planning

#endif  // MOTION_PLANNING_PLAN_ENVRIONMENT_H_