/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 14:18:05
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 18:16:01
 */
#include <stdint.h>

#ifndef __LOCAL_PLANNER_INTERFACE_H__
#define __LOCAL_PLANNER_INTERFACE_H__
#include "common/common.h"
#include "local_planner_config.h"
#include "plan_environment/plan_environment.h"
namespace motion_planning {
namespace local_planner {
// 局部规划的接口，
using namespace common;
using namespace plan_environment;
class LocalPlannerInterface {

public:
  //局部规划的接口，输入实时的环境，全局轨迹，车辆定位，输出控制信息，优化后的轨迹
  //    输入的环境是车体坐标系下的环境，如果原始环境是激光雷达坐标系下，需要进行转换
  virtual bool plan(const PlanEnvrionment::Ptr &plan_env,
                    const VehicleState &current_pose, const Path &path,
                    Trajectory &traj) = 0;
  // 局部规划的参数配置，具体参数，根据算法再增加
  void setConfig(const LocalPlannerConfig &cfg) {
    cfg_ = cfg;
    set_config_flag_ = true;
  }
  typedef std::unique_ptr <LocalPlannerInterface> Ptr;

protected:
  LocalPlannerConfig cfg_;
  bool set_config_flag_ = false;
};
} // namespace local_planner
} // namespace motion_planning

#endif /* __LOCAL_PLANNER_INTERFACE_H__ */
