/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-28 10:06:14
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-28 13:22:55
 */
#include <iostream>

using namespace std;

#include "voronoi_graph.h"

namespace motion_planning {
namespace plan_environment {

void VoronoiGraph::calcSegments(std::vector<Segment> &segs, cv::Mat &_map,
                                float *potential, float _path_length,
                                float _optimizeCrossingPixels,
                                float _optimizeEndSegmentsPixel) {

  cv::Mat distField, voronoiMap;
  cv::flip(_map, _map, 0);
  // cv::imshow("raw", _map);
  // cv::waitKey(0);
  cv::Mat mat_color_inversion = _map.clone();
  for (int i = 0; i < _map.rows; i++) {
    for (int j = 0; j < _map.cols; j++) {
      if (_map.at<uchar>(i, j) >= 100) {
        mat_color_inversion.at<uchar>(i, j) = 0;
      } else {
        mat_color_inversion.at<uchar>(i, j) = 100;
      }
    }
  }
  // cv::imshow("bitwise_not", _map);
  // cv::waitKey(0);
  // cv::imshow("bitwise_not", _map);
  // cv::waitKey(0);
  // 1.获取距离场mat
  computeDistanceField(mat_color_inversion, distField);

  //  normalize(distField, distField, 0, 1.0, NORM_MINMAX);
  // cv::flip(distField, distField, 0);
  // cv::imshow("distField", distField);
  // cv::waitKey(0);

  //   计算维诺图
  computeVoronoiMap(distField, voronoiMap);
  // cv::imshow("voronoiMap", voronoiMap);
  // cv::waitKey(0);

  Segment_Expander exp;
  exp.Initialize(_map, distField, voronoiMap);
  std::vector<std::vector<Eigen::Vector2d>> points =
      exp.calcEndpoints(potential);
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> segments;
  int nx = _map.cols;
  int ny = _map.rows;
  exp.Initialize(_map, distField, voronoiMap);
  std::fill(potential, potential + nx * ny, -1);
  segs = exp.getGraph(points, potential, _path_length, _optimizeCrossingPixels,
                      _optimizeEndSegmentsPixel);
  for (uint32_t i = 0; i < segments.size(); i++) {
    std::vector<uint32_t> predecessors = segs[i].getPredecessors();
    std::vector<uint32_t> successors = segs[i].getSuccessors();
  }
}

void VoronoiGraph::computeDistanceField(const cv::Mat &_map,
                                        cv::Mat &_distField) {
  cv::distanceTransform(_map, _distField, cv::DIST_L2, 3);
}
void VoronoiGraph::computeVoronoiMap(const cv::Mat &_distField,
                                     cv::Mat &_voronoiMap) {
  Mat srcMap = _distField;

  srcMap.convertTo(_voronoiMap, CV_8UC1, 0.0);
  voronoi_map::greyscale_thinning(srcMap, _voronoiMap);
  cv::threshold(_voronoiMap, _voronoiMap, 1, 255, cv::THRESH_BINARY);
  voronoi_map::sceletonize(_voronoiMap, _voronoiMap);
}

void VoronoiGraph::getJsonSegments(Json::Value &json_segments,
                                   const std::vector<Segment> &segments,
                                   const Eigen::Vector2d &origin,
                                   const double &resolution) {
  if (segments.size() == 0) {
    return;
  }
  json_segments.clear();
  Json::Value json_segment;
  Json::Value point;
  for (auto segment : segments) {
    json_segment["id"] = segment.getId();
    json_segment["weight"] = segment.getLength();
    json_segment["min_path_space"] = segment.getMinPathSpace();
    auto start = getCartesianCoordinate(segment.getStart(), origin, resolution);
    point["x"] = start[0];
    point["y"] = start[1];
    json_segment["start"] = point;
    auto end = getCartesianCoordinate(segment.getEnd(), origin, resolution);
    point["x"] = end[0];
    point["y"] = end[1];
    json_segment["end"] = point;
    Json::Value json_points;
    for (auto segment_point : segment.getPath()) {
      auto pt = getCartesianCoordinate(segment_point, origin, resolution);
      point["x"] = pt[0];
      point["y"] = pt[1];
      json_points.append(point);
    }
    json_segment["points"] = json_points;
    Json::Value json_predecessors;
    for (auto predecessor : segment.getPredecessors()) {
      json_predecessors.append(predecessor);
    }
    json_segment["predecessors"] = json_predecessors;
    Json::Value json_successors;
    for (auto suc : segment.getSuccessors()) {
      json_successors.append(suc);
    }
    json_segment["successors"] = json_successors;
    json_segments.append(json_segment);
  }
}
} // namespace plan_environment
} // namespace motion_planning