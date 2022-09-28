/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-08 10:53:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 10:02:35
 */

#include "crossing.h"
#include <limits>

namespace motion_planning {
namespace plan_environment {
Crossing::Crossing(const std::vector<Eigen::Vector2d> &_segment_points)
    : segments_start_(), segments_end_() {
  int min_x = __INT_MAX__;
  int max_x = 0;
  int min_y = __INT_MAX__;
  int max_y = 0;

  for (auto it = _segment_points.begin(); it != _segment_points.end(); ++it) {
    if ((*it)[0] > max_x)
      max_x = (*it)[0];

    if ((*it)[0] < min_x)
      min_x = (*it)[0];

    if ((*it)[1] > max_y)
      max_y = (*it)[1];

    if ((*it)[1] < min_y)
      min_y = (*it)[1];
  }

  surrounding_points_ = _segment_points;
  center_[0] = ((float)(max_x + min_x)) / 2.0;
  center_[1] = ((float)(max_y + min_y)) / 2.0;
}

bool Crossing::tryAddSegment(Segment &_seg) {
  for (const Eigen::Vector2d &srPoint : surrounding_points_) {
    if (srPoint[0] == _seg.getStart()[0] && srPoint[1] == _seg.getStart()[1]) {
      for (uint32_t &s_idx : segments_start_) {
        _seg.addPredecessor(s_idx);
        (*segment_reference_)[s_idx].addPredecessor(_seg.getId());
      }

      for (uint32_t &s_idx : segments_end_) {
        _seg.addPredecessor(s_idx);
        (*segment_reference_)[s_idx].addSuccessor(_seg.getId());
      }

      std::vector<Eigen::Vector2d> p = _seg.getPath();
      p.insert(p.begin(), center_);
      _seg.setPath(p);
      segments_start_.push_back(_seg.getId());
      return true;
    }

    if (srPoint[0] == _seg.getEnd()[0] && srPoint[1] == _seg.getEnd()[1]) {
      for (uint32_t &s_idx : segments_start_) {
        _seg.addSuccessor(s_idx);
        (*segment_reference_)[s_idx].addPredecessor(_seg.getId());
      }

      for (uint32_t &s_idx : segments_end_) {
        _seg.addSuccessor(s_idx);
        (*segment_reference_)[s_idx].addSuccessor(_seg.getId());
      }

      std::vector<Eigen::Vector2d> p = _seg.getPath();
      p.push_back(center_);
      _seg.setPath(p);
      segments_end_.push_back(_seg.getId());
      return true;
    }
  }

  return false;
}

Eigen::Vector2d Crossing::getCenter() const { return center_; }

void Crossing::setSegmentReference(
    const std::shared_ptr<std::vector<Segment>> &segs) {
  segment_reference_ = segs;
}

} // namespace plan_environment
} // namespace motion_planning
