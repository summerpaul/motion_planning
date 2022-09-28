/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 17:09:32
 */
#ifndef MOTION_PLANNING_COMMON_H_
#define MOTION_PLANNING_COMMON_H_
#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'
#define inf 1 >> 30

namespace motion_planning {
namespace common {

struct VehicleState {
  Eigen::Vector2d position = Eigen::Vector2d::Zero(); // x y
  Eigen::Vector2d speed = Eigen::Vector2d::Zero();    // vx vy
  Eigen::Vector2d acc = Eigen::Vector2d::Zero();      // ax ay
  double angular_speed = 0;
  double angular_acc = 0;
  int prim = -1; // the motion primitive of the node
  double theta = 0;
  double kappa = 0;
};

struct StampedVehicleState {
  VehicleState pose;
  double time;
};
typedef std::vector<VehicleState> Path;
typedef std::vector<StampedVehicleState> Trajectory;
;

struct Node {
  VehicleState state;
  Eigen::Vector2i index;
  Eigen::Vector2d input;
  int motion_index;
  double duration;
  double g_score;
  double f_score;
  std::shared_ptr<Node> parent;
  char node_state;
  double time;
  int time_idx;
  Node() {
    parent = nullptr;
    node_state = NOT_EXPAND;
  }
  ~Node() {}
};
typedef std::shared_ptr<Node> NodePtr;

template <typename T> struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const &matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

class NodeHashTable {
private:
  std::unordered_map<Eigen::Vector2i, NodePtr, matrix_hash<Eigen::Vector2i>>
      data_2d_;
  std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash<Eigen::Vector3i>>
      data_3d_;

public:
  void insert(Eigen::Vector2i idx, NodePtr node) {
    data_2d_.insert(std::make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, NodePtr node) {
    data_3d_.insert(
        std::make_pair(Eigen::Vector3i(idx(0), idx(1), time_idx), node));
  }

  NodePtr find(Eigen::Vector2i idx) {
    auto iter = data_2d_.find(idx);
    return iter == data_2d_.end() ? NULL : iter->second;
  }
  NodePtr find(Eigen::Vector2i idx, int time_idx) {
    auto iter = data_3d_.find(Eigen::Vector3i(idx(0), idx(1), time_idx));
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  void clear() {
    data_3d_.clear();
    data_2d_.clear();
  }
};

class NodeComparator {
public:
  bool operator()(NodePtr node1, NodePtr node2) {
    return node1->f_score > node2->f_score;
  }
};

static double getDiagHeu(const Eigen::Vector2d &x1, const Eigen::Vector2d &x2,
                         const double &tie_breaker = 1.001) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double h = (dx + dy) + (sqrt(2) - 2) * std::min(dx, dy);
  return tie_breaker * h;
}
static double getManhHeu(const Eigen::Vector2d &x1, const Eigen::Vector2d &x2,
                         const double &tie_breaker = 1.001) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  return tie_breaker * (dx + dy);
}
static double getEuclHeu(const Eigen::Vector2d &x1, const Eigen::Vector2d &x2,
                         const double &tie_breaker = 1.001) {
  return tie_breaker * (x2 - x1).norm();
}
} // namespace common
} // namespace motion_planning
#endif