/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 15:19:40
 */
#include "astar_search_grid_map.h"

namespace motion_planning {
namespace global_planner {
AStarSearchGridMap::AStarSearchGridMap() {
  motions_.push_back(Eigen::Vector2i(-1, -1));
  motions_.push_back(Eigen::Vector2i(-1, 0));
  motions_.push_back(Eigen::Vector2i(-1, 1));
  motions_.push_back(Eigen::Vector2i(0, -1));
  motions_.push_back(Eigen::Vector2i(0, 1));
  motions_.push_back(Eigen::Vector2i(1, -1));
  motions_.push_back(Eigen::Vector2i(1, 0));
  motions_.push_back(Eigen::Vector2i(1, 1));
}

void AStarSearchGridMap::setPlanEnvrionment(
    const PlanEnvrionment::Ptr &plan_env) {
  grid_map_ptr_ = plan_env->getGridMap();
  std::cout << "setGridMap " << "!global_gridmap_ptr_ is " << !plan_env->getGridMap() << std::endl;
  if (grid_map_ptr_) {
    std::cout << "setPlanEnvrionment in AStarSearchGridMap " << std::endl;
  }
}
int AStarSearchGridMap::search(const VehicleState &start_pt,
                               const VehicleState &end_pt) {
  if (!grid_map_ptr_) {
    std::cout << "grid map is null, fail to plan!!" << std::endl;
    return NO_PATH;
  }
  double resolution = grid_map_ptr_->getResolution();
  std::cout << "start astar search with grid map" << std::endl;
  end_pt_ = end_pt;
  NodePtr cur_node = std::make_shared<Node>();
  cur_node->parent = NULL;
  cur_node->state = start_pt;
  cur_node->index = grid_map_ptr_->getGridMapIndex(start_pt.position);
  cur_node->g_score = 0.0;
  cur_node->f_score = 5.0 * getDiagHeu(start_pt.position, end_pt.position);
  std::cout << "f_score is " << cur_node->f_score << std::endl;
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  Eigen::Vector2i end_index = grid_map_ptr_->getGridMapIndex(end_pt.position);
  expanded_nodes_.insert(cur_node->index, cur_node);
  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;
  double tmp_g_score;
  double tmp_f_score;
  while (!open_set_.empty()) {
    cur_node = open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
                     abs(cur_node->index(1) - end_index(1)) <= 1;
    if (reach_end) {
      terminate_node = cur_node;
      std::cout << "terminate_node pose is " << terminate_node->state.position
                << std::endl;
      retrievePath(terminate_node);
      return REACH_END;
    }
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    // 拓展节点
    for (auto motion : this->motions_) {
      //拓展后节点的id
      auto neighbor_index = cur_node->index + motion;
      //   判断节点是否在地图中
      if (!grid_map_ptr_->isVerify(neighbor_index))
        continue;
      if (grid_map_ptr_->isOccupied(neighbor_index))
        continue;
      neighbor = expanded_nodes_.find(neighbor_index);
      if (neighbor != nullptr && neighbor->node_state == IN_CLOSE_SET)
        continue;
      double x = motion[0] * resolution;
      double y = motion[1] * resolution;
      tmp_g_score = sqrt(x * x + y * y) + cur_node->g_score;
      tmp_f_score =
          tmp_g_score + 5.0 * getDiagHeu(grid_map_ptr_->getCartesianCoordinate(
                                             neighbor_index),
                                         end_pt.position);
      if (neighbor == nullptr) {
        neighbor = std::make_shared<Node>();
        neighbor->state.position =
            grid_map_ptr_->getCartesianCoordinate(neighbor_index);
        neighbor->index = neighbor_index;
        neighbor->f_score = tmp_f_score;
        neighbor->g_score = tmp_g_score;
        neighbor->parent = cur_node;
        neighbor->node_state = IN_OPEN_SET;
        open_set_.push(neighbor);
        expanded_nodes_.insert(neighbor_index, neighbor);
        // std::cout << "neighbor->state.position is " <<
        // neighbor->state.position << std::endl;

      } else if (neighbor->node_state == IN_OPEN_SET) {
        neighbor->parent = cur_node;
        neighbor->f_score = tmp_f_score;
        neighbor->g_score = tmp_g_score;
      } else {
        std::cout << "error type in searching: " << neighbor->node_state
                  << std::endl;
      }
    }
  }
  return NO_PATH;
}
std::vector<VehicleState> AStarSearchGridMap::getPath(const double &delta_t) {
  std::vector<VehicleState> path;
  for (int i = 0; i < path_nodes_.size(); ++i) {
    path.push_back(path_nodes_[i]->state);
  }
  path.push_back(end_pt_);
  return path;
}
void AStarSearchGridMap::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();
  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator>
      empty_queue;
  open_set_.swap(empty_queue);
}
void AStarSearchGridMap::retrievePath(NodePtr end_node) {
  NodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
} // namespace global_planner
} // namespace motion_planning