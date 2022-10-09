/**
 * @Author: Yunkai Xia
 * @Date:   2022-10-09 10:50:13
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 18:56:54
 */
#include <stdint.h>

#ifndef __PATH_OPTIMIZER_H__
#define __PATH_OPTIMIZER_H__

#include "common/spline.h"
#include <functional>
#include <nlopt/nlopt.hpp>
#include <vector>
namespace motion_planning ::global_planner {
using ObjectiveFunc =
    std::function<double(const std::vector<double> &, std::vector<double> &)>;
class PathOptimizer {

public:
  nlopt::result optimize() {
    nlopt::result result = nlopt::result::FAILURE;
    return result;
  }

  // 目标函数
  double objectiveFunction(const std::vector<double> &x,
                           std::vector<double> &grad) {}

  // 获取优化后的曲线
  Spline2D getOptimizeSpline() {
    std::vector<double> path_x, path_y;
    for (size_t i = 0; i < param_.size() / 2; i += 2) {
      path_x.push_back(param_[i]);
      path_y.push_back(param_[i + 1]);
    }
    return Spline2D(path_x, path_y);
  }

  std::vector<double> param_; //优化参数，二维的xy点集合
};
} // namespace motion_planning::global_planner
#endif /* __PATH_OPTIMIZER_H__ */
