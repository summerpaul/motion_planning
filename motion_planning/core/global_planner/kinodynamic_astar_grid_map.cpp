/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-27 15:04:04
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 17:15:23
 */
#include <iostream>

#include "kinodynamic_astar_grid_map.h"
namespace motion_planning {
namespace global_planner {
KinodynamicAstarGridMap::KinodynamicAstarGridMap() {}
int KinodynamicAstarGridMap::search(const VehicleState &start_pt,
                                    const VehicleState &end_pt) {
  if (!grid_map_ptr_) {
    std::cout << "grid map is null, fail to plan!!" << std::endl;
    return NO_PATH;
  }
  start_val_ = start_pt.speed;
  start_acc_ = start_pt.acc;
  double resolution = grid_map_ptr_->getResolution();
  std::cout << "start kinodynamic search with grid map" << std::endl;
  NodePtr cur_node = std::make_shared<Node>();
  cur_node->parent = NULL;
  cur_node->state = start_pt;
  cur_node->index = grid_map_ptr_->getGridMapIndex(start_pt.position);
  cur_node->g_score = 0.0;
  return NO_PATH;
}
std::vector<VehicleState>
KinodynamicAstarGridMap::getPath(const double &delta_t = 0.1) {}
void KinodynamicAstarGridMap::reset() {}

void KinodynamicAstarGridMap::setPlanEnvrionment(
    const PlanEnvrionment::Ptr &plan_env) {
  grid_map_ptr_ = plan_env->getGridMap();
}

// obvp问题：Optical Boundary Value Problem ，mininum jerk
// 主要原理是利用庞特里亚金原理解决两点边值问题，得到最优解后用最优解的控制代价作为启发函数。
void KinodynamicAstarGridMap::estimateHeuristic(const VehicleState &currt_pt,
                                                const VehicleState &target_pt,
                                                double &optical_time) {
  // 使用五次多项式

  // 起点终点的位置向量
  const Eigen::Vector2d dp = target_pt.position - currt_pt.position;
  // 当前速度
  const Vector2d v0 = currt_pt.speed;
  const Vector2d v1 = target_pt.speed;
  //   对启发函数进行时间求导，得到关于时间的四次多项式，
  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = 10; //时间花费的权重
  // 关于时间的一元四次方程是通过费拉里方法求解的，需要嵌套一个元三次方程进行求解，也就是代码中应的cubic（）函数
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);
  double v_max = 1.5 * 0.5; // 1.5表示最大速度
  double t_bar = (v0 - v1).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);
  double cost = 100000000;
  double t_d = t_bar;
  or (auto t : ts) {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost) {
      cost = c;
      t_d = t;
    }
  }
  optimal_time = t_d;
  return 1.0 * (1 + 0.001) * cost;
}

vector<double> KinodynamicAstarGridMap::quartic(double a, double b, double c,
                                                double d, double e) {
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys =
      cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0) {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 +
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 -
             0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  } else {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D)) {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E)) {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}
vector<double> KinodynamicAstarGridMap::cubic(double a, double b, double c,
                                              double d) {
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0) {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  } else if (D == 0) {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  } else {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

} // namespace global_planner
} // namespace motion_planning