/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 14:31:32
 */
#ifndef MOTION_PLANNING_PLAN_ENVRIONMENT_H_
#define MOTION_PLANNING_PLAN_ENVRIONMENT_H_
#include "esdf_map.h"
#include "grid_map.h"
namespace motion_planning {
namespace plan_environment {
class PlanEnvrionment {
public:
  void setGridMap(const GridMap::Ptr &grid_map) {
    if (!grid_map) {
      return;
    }
    grid_map_ptr_ = grid_map;
  }
  GridMap::Ptr getGridMap() const { return grid_map_ptr_; }
  void setESDFMap(const GridMap::Ptr &grid_map) {
    if (!grid_map) {
      return;
    }
    esdf_map_ptr_ = std::make_shared<ESDFMap>();
    esdf_map_ptr_->updateGridmap(grid_map);
    esdf_map_ptr_->updateESDF2d();
  }

  ESDFMap::Ptr getESDFMap() const { return esdf_map_ptr_; }

private:
  GridMap::Ptr grid_map_ptr_;
  ESDFMap::Ptr esdf_map_ptr_;

public:
  typedef std::shared_ptr<PlanEnvrionment> Ptr;
};
} // namespace plan_environment
} // namespace motion_planning

#endif // MOTION_PLANNING_PLAN_ENVRIONMENT_H_