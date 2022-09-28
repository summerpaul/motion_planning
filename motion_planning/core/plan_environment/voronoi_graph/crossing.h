/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-08 10:53:00
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 10:02:39
 */

#ifndef CROSSING_H
#define CROSSING_H

#include "segment.h"

namespace motion_planning {
namespace plan_environment {
class Crossing {
public:
  Crossing(const std::vector<Eigen::Vector2d> &_segment_points);
  bool tryAddSegment(Segment &_seg);
  Eigen::Vector2d getCenter() const;
  void setSegmentReference(const std::shared_ptr<std::vector<Segment>> &segs);

private:
  std::vector<Eigen::Vector2d> surrounding_points_;
  std::vector<uint32_t> segments_start_;
  std::vector<uint32_t> segments_end_;
  Eigen::Vector2d center_;
  std::shared_ptr<std::vector<Segment>> segment_reference_;
};
} // namespace plan_environment
} // namespace motion_planning_core
#endif // PLANNER_NODE_H
