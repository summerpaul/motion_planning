/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-27 15:04:04
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-09 18:25:41
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
  Eigen::Vector2i end_index = grid_map_ptr_->getGridMapIndex(end_pt.position);
  Eigen::Vector2i start_index =
      grid_map_ptr_->getGridMapIndex(start_pt.position);

  if (grid_map_ptr_->isOccupied(end_index) ||
      !grid_map_ptr_->isVerify(end_index)) {
    std::cout << "invalid goal point , fail to plan!!" << std::endl;
    return NO_PATH;
  }
  if (grid_map_ptr_->isOccupied(start_index) ||
      !grid_map_ptr_->isVerify(start_index)) {
    std::cout << "invalid start point , fail to plan!!" << std::endl;
    return NO_PATH;
  }
  double lambda_heu = 5.0;
  start_val_ = start_pt.speed;
  start_acc_ = start_pt.acc;
  double resolution = grid_map_ptr_->getResolution();
  std::cout << "start kinodynamic search with grid map" << std::endl;
  NodePtr cur_node = std::make_shared<Node>();
  cur_node->parent = NULL;
  cur_node->state = start_pt;
  cur_node->index = grid_map_ptr_->getGridMapIndex(start_pt.position);
  cur_node->g_score = 0.0;
  double time_to_goal;
  double cost_to_goal = estimateHeuristic(start_pt, end_pt, time_to_goal);
  // double cost_to_goal = getEuclHeu(start_pt.position, end_pt.position);
  cur_node->node_state = IN_OPEN_SET;
  cur_node->f_score = lambda_heu * cost_to_goal;
  open_set_.push(cur_node);
  expanded_nodes_.insert(cur_node->index, cur_node);
  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;
  std::vector<NodePtr> tmp_expand_nodes;
  const int tolerance = ceil(1 / resolution);
  while (!open_set_.empty()) {
    cur_node = open_set_.top();

    bool near_end = fabs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    fabs(cur_node->index(1) - end_index(1)) <= tolerance;
    // 接近终点以后，需要使用一条曲线进行j连接
    // std::cout << "dist to goal is "
    //           << (cur_node->state.position - end_pt.position).norm()
    //           << std::endl;
    if (near_end) {
      std::cout << "near_end " << std::endl;
      
      estimateHeuristic(cur_node->state, end_pt, time_to_goal);
      if(!computeShotTraj(cur_node->state, end_pt, time_to_goal)){
        std::cout << "some thing wrong with end ShotTraj " << std::endl;
        return NO_PATH;
      }
      terminate_node = cur_node;
      retrievePath(terminate_node);
      return REACH_END;
    }
    // 弹出最小的节点
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;

    double res = 1 / 2.0, time_res = 1 / 1.0;
    auto cur_state = cur_node->state;
    VehicleState pro_state;              //拓展的状态
    std::vector<Eigen::Vector2d> inputs; //输入是二维度的加速度
    Eigen::Vector2d um;
    // 离散时间
    std::vector<double> durations;
    for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
      for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res) {
        um << ax, ay;
        inputs.push_back(um);
      }
    for (double tau = time_res * max_tau_; tau <= max_tau_;
         tau += time_res * max_tau_)
      durations.push_back(tau);
    // 对状态进行拓展
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j) {
        // std::cout << "um is " << um << std::endl;
        um = inputs[i];
        double tau = durations[j];
        // 将装填进行拓展
        stateTransit(cur_state, pro_state, um, tau);
        // std::cout << "pro state position is " << pro_state.position <<
        // std::endl;
        //拓展的位置在栅格地图中的坐标
        auto neighbor_index =
            grid_map_ptr_->getGridMapIndex(pro_state.position);
        if (!grid_map_ptr_->isVerify(neighbor_index))
          continue;
        if (grid_map_ptr_->isOccupied(neighbor_index))
          continue;
        // 查找节点是否在扩展节点中
        neighbor = expanded_nodes_.find(neighbor_index);
        // std::cout << "!neighbor is " << !neighbor << std::endl;
        //
        if (neighbor != nullptr && neighbor->node_state == IN_CLOSE_SET)
          continue; // 当拓展的节点在close set中，跳过
        // 速度与加速度检测
        // std::cout << "check speed " << std::endl;
        if (fabs(pro_state.speed(0)) > max_vel_ ||
            fabs(pro_state.speed(1)) > max_vel_) {
          continue;
        }
        // Check not in the same voxel
        // std::cout << "Check not in the same voxel " << std::endl;
        auto diff = pro_state.position - cur_state.position;
        if (diff.norm() < grid_map_ptr_->getResolution()) {
          continue;
        }
        // Check safety
        // std::cout << "Check safety " << std::endl;
        Eigen::Vector2i pos_index;
        VehicleState xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k) {
          double dt = tau * double(k) / double(check_num_);
          stateTransit(cur_state, xt, um, dt);
          pos_index = grid_map_ptr_->getGridMapIndex(xt.position);
          if (grid_map_ptr_->isOccupied(pos_index)) {
            is_occ = true;
            break;
          }
        }
        if (is_occ) {
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        // tmp_g_score = (0.5 * (um)*pow(tau, 2)).norm() + cur_node->g_score;
        tmp_g_score = (um.squaredNorm() + lambda_heu) +  cur_node->g_score;
        // tmp_f_score = tmp_g_score + lambda_heu *
        // getEuclHeu(pro_state.position,
        //                                                     end_pt.position);

        tmp_f_score =
            tmp_g_score +
            lambda_heu * estimateHeuristic(pro_state, end_pt, time_to_goal);
        // std::cout << "prune " << std::endl;

        bool prune = false;
        //
        for (int j = 0; j < tmp_expand_nodes.size(); ++j) {
          // std::cout << "tmp_expand_nodes.size() size is "
          //           << tmp_expand_nodes.size() << std::endl;
          NodePtr expand_node = tmp_expand_nodes[j];
          if ((neighbor_index - expand_node->index).norm() == 0) {
            prune = true;
            if (tmp_f_score < expand_node->f_score) {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
            }
            break;
          }
        }
        if (!prune) {
          // 扩展新的节点
          if (neighbor == NULL) {
            neighbor = std::make_shared<Node>();
            neighbor->index = neighbor_index;
            neighbor->state = pro_state;
            neighbor->f_score = tmp_f_score;
            neighbor->g_score = tmp_g_score;
            neighbor->input = um;
            neighbor->duration = tau;
            neighbor->parent = cur_node;
            neighbor->node_state = IN_OPEN_SET;

            open_set_.push(neighbor);
            expanded_nodes_.insert(neighbor_index, neighbor);
            tmp_expand_nodes.push_back(neighbor);
          }
          // 更新节点
          else if (neighbor->node_state == IN_OPEN_SET) {
            if (tmp_g_score < neighbor->g_score) {
              neighbor->state = pro_state;
              neighbor->f_score = tmp_f_score;
              neighbor->g_score = tmp_g_score;
              neighbor->input = um;
              neighbor->duration = tau;
            }
          } else {
            std::cout << "error type in searching: " << neighbor->node_state
                      << std::endl;
          }
        }
      }
  }
  return NO_PATH;
}
std::vector<VehicleState>
KinodynamicAstarGridMap::getPath(const double &delta_t) {
  std::cout << "path_nodes_ size is " << path_nodes_.size() << std::endl;

  auto node = path_nodes_.back();
  Path path;
  VehicleState x0, xt;
  while (node->parent != nullptr) {
    auto ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;
    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      path.push_back(xt);
    }
    node = node->parent;
  }
  reverse(path.begin(), path.end());

  //
  VehicleState x_shot;
  std::cout << "get short path " << std::endl;
  if (is_shot_succ_) {
    Eigen::Vector2d coord;
    Eigen::VectorXd poly1d, time(4);
    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 2; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      x_shot.position = coord;
      path.push_back(x_shot);
    }
  }
  return path;
}
void KinodynamicAstarGridMap::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
void KinodynamicAstarGridMap::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator>
      empty_queue;
  open_set_.swap(empty_queue);
}

void KinodynamicAstarGridMap::setPlanEnvrionment(
    const PlanEnvrionment::Ptr &plan_env) {
  grid_map_ptr_ = plan_env->getGridMap();
}
void KinodynamicAstarGridMap::stateTransit(VehicleState &current_state,
                                           VehicleState &pro_state,
                                           const Eigen::Vector2d &um,
                                           const double &tau) {
  pro_state.speed = current_state.speed + tau * um;
  Eigen::Vector2d dp = 0.5 * (pro_state.speed + current_state.speed) * tau;
  pro_state.position = current_state.position + dp;
}
bool KinodynamicAstarGridMap::computeShotTraj(const VehicleState &state1,
                                              const VehicleState &state2,
                                              const double &time_to_goal) {
  const Eigen::Vector2d p0 = state1.position;
  const Eigen::Vector2d dp = state2.position - p0; //起点与终点向量
  const Eigen::Vector2d v0 = state1.speed;
  const Eigen::Vector2d v1 = state2.speed;
  const Eigen::Vector2d dv = v1 - v0;
  double t_d = time_to_goal;
  Eigen::MatrixXd coef(2, 4);
  end_vel_ = v1;
  Eigen::Vector2d a =
      1.0 / 6.0 *
      (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Eigen::Vector2d b =
      0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Eigen::Vector2d c = v0;
  Eigen::Vector2d d = p0;
  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;
  Eigen::Vector2d coord, vel, acc;
  Eigen::VectorXd poly1d, t, polyv, polya;
  Eigen::Vector2i index;
  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;
  /* ---------- forward checking of trajectory ---------- */
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = Eigen::VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 2; dim++) {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);
    }
    index = grid_map_ptr_->getGridMapIndex(coord);
    if (!grid_map_ptr_->isVerify(index)) {
      return false;
    }
    if (grid_map_ptr_->isOccupied(index)) {
      return false;
    }
  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

// obvp问题：Optical Boundary Value Problem ，mininum jerk
// 主要原理是利用庞特里亚金原理解决两点边值问题，得到最优解后用最优解的控制代价作为启发函数。
double KinodynamicAstarGridMap::estimateHeuristic(const VehicleState &currt_pt,
                                                  const VehicleState &target_pt,
                                                  double &optimal_time) {
  // 使用五次多项式

  // 起点终点的位置向量
  const double w_time = 10;
  const Eigen::Vector2d dp = target_pt.position - currt_pt.position;
  // 当前速度
  const Eigen::Vector2d v0 = currt_pt.speed;
  const Eigen::Vector2d v1 = target_pt.speed;
  //   对启发函数进行时间求导，得到关于时间的四次多项式，
  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time; //时间花费的权重
  // 关于时间的一元四次方程是通过费拉里方法求解的，需要嵌套一个元三次方程进行求解，也就是代码中应的cubic（）函数
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);
  double v_max = max_vel_ * 0.5; // 1.5表示最大速度
  double t_bar = (v0 - v1).lpNorm<Eigen::Infinity>() / v_max;
  ts.push_back(t_bar);
  double cost = 100000000;
  double t_d = t_bar;
  for (auto t : ts) {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time * t;
    if (c < cost) {
      cost = c;
      t_d = t;
    }
  }
  optimal_time = t_d;
  // return 1.0 * (1 + 0.001) * cost;
  return cost;
}

std::vector<double> KinodynamicAstarGridMap::quartic(double a, double b,
                                                     double c, double d,
                                                     double e) {
  std::vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  std::vector<double> ys =
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
std::vector<double> KinodynamicAstarGridMap::cubic(double a, double b, double c,
                                                   double d) {
  std::vector<double> dts;

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