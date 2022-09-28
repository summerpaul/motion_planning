/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 09:35:56
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 13:24:05
 */
#include <stdint.h>

#ifndef __VORONOI_ROADMAP_H__
#define __VORONOI_ROADMAP_H__
#include "segment.h"
#include "segment_expander.h"
#include "thinning.h"
#include <jsoncpp/json/json.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
namespace motion_planning {
namespace plan_environment {
using namespace cv;
class VoronoiGraph {
public:
  // 输入是open cvmat（栅格地图）
  void calcSegments(std::vector<Segment>& segs, cv::Mat &_map, float *potential,
                                    float _path_length,
                                    float _optimizeCrossingPixels,
                                    float _optimizeEndSegmentsPixel);
  void getJsonSegments(Json::Value &json_segments,
                       const std::vector<Segment> &segments,
                       const Eigen::Vector2d &origin, const double &resolution);

private:
  void computeDistanceField(const cv::Mat &_map, cv::Mat &_distField);
  void computeVoronoiMap(const cv::Mat &_distField, cv::Mat &_voronoiMap);
  Eigen::Vector2d getCartesianCoordinate(const Eigen::Vector2d &index,
                                         const Eigen::Vector2d &origin,
                                         const double &resolution) const {
    Eigen::Vector2d point;
    point[0] = static_cast<double>(index[0] * resolution);
    point[1] = static_cast<double>(index[1] * resolution);
    return calcOldCoordPos(origin, point);
  }
  static Eigen::Vector2d
  calcOldCoordPos(const Eigen::Vector2d &new_coord_origin,
                  const Eigen::Vector2d &pos) {
    Eigen::Vector2d old_position;
    old_position[0] = new_coord_origin[0] + pos[0] * cos(0) - pos[1] * sin(0);
    old_position[1] = new_coord_origin[1] + pos[0] * sin(0) + pos[1] * cos(0);
    return old_position;
  }
};
} // namespace plan_environment
} // namespace motion_planning

#endif /* __VORONOI_ROADMAP_H__ */
