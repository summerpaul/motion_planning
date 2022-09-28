/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-08 10:53:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 10:02:52
 */

#include "segment.h"
#include <limits>

namespace motion_planning {
namespace plan_environment {
uint32_t Segment::static_id_ = 0;

void Segment::cleanNeighbors(uint32_t _id) {
  for (uint32_t i = 0; i < predecessor_.size(); i++) {
    if (predecessor_[i] == _id) {
      predecessor_.erase(predecessor_.begin() + i);
    }
  }

  for (uint32_t i = 0; i < successor_.size(); i++) {
    if (successor_[i] == _id) {
      successor_.erase(successor_.begin() + i);
    }
  }
}

void Segment::decreaseNeighborIdAbove(uint32_t _id) {
  if (id_ >= _id)
    id_--;

  for (uint32_t i = 0; i < predecessor_.size(); i++) {
    if (predecessor_[i] >= _id) {
      predecessor_[i]--;
    }
  }

  for (uint32_t i = 0; i < successor_.size(); i++) {
    if (successor_[i] >= _id) {
      successor_[i]--;
    }
  }
}

void Segment::addPredecessor(const uint32_t _predecessor) {
  predecessor_.push_back(_predecessor);
}
void Segment::addSuccessor(const uint32_t _successor) {
  successor_.push_back(_successor);
}
Segment::Segment(const std::vector<Eigen::Vector2d> &_points,
                 const float _min_space)
    : predecessor_(), successor_(), optimized_start_(false),
      optimized_end_(false) {
  if (_points.size() > 0) {
    start_ = _points.front();
    end_ = _points.back();
    length_ = _points.size();
    min_space_ = _min_space;
    way_points_ = _points;
  }

  id_ = static_id_++;
}
Segment::Segment(const uint32_t _id,
                 const std::vector<Eigen::Vector2d> &_points,
                 const float _min_space)
    : predecessor_(), successor_(), optimized_start_(false),
      optimized_end_(false) {
  if (_points.size() > 0) {
    start_ = _points.front();
    end_ = _points.back();
    length_ = _points.size();
    min_space_ = _min_space;
    way_points_ = _points;
  }

  id_ = _id;
}
void Segment::setStart(const Eigen::Vector2d &_pt) {
  if (way_points_.size() == 0)
    way_points_.emplace_back(_pt);
  way_points_[0] = _pt;
  start_ = _pt;
}
void Segment::setEnd(const Eigen::Vector2d &_pt) {
  while (way_points_.size() <= 1) {
    way_points_.emplace_back(_pt);
  }
  way_points_[way_points_.size() - 1] = _pt;
  end_ = _pt;
}

uint32_t Segment::getId() const { return id_; }

void Segment::setId(uint32_t _id) { id_ = _id; }

const Eigen::Vector2d &Segment::getEnd() const { return end_; }

const Eigen::Vector2d &Segment::getStart() const { return start_; }

const std::vector<uint32_t> &Segment::getPredecessors() const {
  return predecessor_;
}

const std::vector<uint32_t> &Segment::getSuccessors() const {
  return successor_;
}

bool Segment::containsPredecessor(const uint32_t _predecessor) {
  for (const auto &pred : predecessor_) {
    if (pred == _predecessor)
      return true;
  }

  return false;
}

bool Segment::containsSuccessor(const uint32_t _successor) {
  for (uint32_t i = 0; i < successor_.size(); i++) {
    if (successor_[i] == _successor)
      return true;
  }

  return false;
}

void Segment::resetId() { static_id_ = 0; }

std::vector<Eigen::Vector2d> Segment::getPath() const { return way_points_; }

void Segment::setPath(const std::vector<Eigen::Vector2d> &_points) {
  if (_points.size() > 0) {
    start_ = _points.front();
    end_ = _points.back();
    length_ = _points.size();
    way_points_ = _points;
  }
}

float Segment::getMinPathSpace() const { return min_space_; }

void Segment::setMinPathSpace(const float _space) { min_space_ = _space; }

int Segment::getLength() const { return length_; }

bool &Segment::getOptStart() { return optimized_start_; }

bool &Segment::getOptEnd() { return optimized_end_; }
} // namespace plan_environment
} // namespace motion_planning
