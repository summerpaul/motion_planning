/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-27 10:53:16
 */
#ifndef MOTION_PLANNING_ESDF_MAP_H_
#define MOTION_PLANNING_ESDF_MAP_H_
#include "grid_map.h"
namespace motion_planning {
namespace plan_environment {
class ESDFMap {
public:
  bool updateGridmap(const GridMap::Ptr &grid_map) {
    if (!grid_map) {
      std::cout << "gridmap_ is nullptr " << std::endl;
      return false;
    }
    grid_map_ptr_ = grid_map;
    int size = grid_map_ptr_->getWidth() * grid_map_ptr_->getHeight();
    distance_buffer_.reserve(size);
    tmp_buffer_.reserve(size);
    map_size_ = grid_map_ptr_->getMapSize2I();
    // std::cout << "size is " << size << std::endl;
    // std::cout << "map_size_ is " << map_size_ << std::endl;
    return true;
  }

  void updateESDF2d() {
    assert(grid_map_ptr_ != nullptr);
    for (int x = 0; x < grid_map_ptr_->getWidth(); x++) {
      fillESDF(
          [&](int y) {
            return grid_map_ptr_->isOccupied(Eigen::Vector2i(x, y)) == 1
                       ? 0
                       : std::numeric_limits<double>::max();
          },
          [&](int y, double val) {
            tmp_buffer_[grid_map_ptr_->getIndex(Eigen::Vector2i(x, y))] = val;
          },
          0, grid_map_ptr_->getHeight() - 1, 1);
    }
    // std::cout << "fill y " << std::endl;
    for (int y = 0; y < grid_map_ptr_->getHeight(); y++) {
      fillESDF(
          [&](int x) {
            return tmp_buffer_[grid_map_ptr_->getIndex(Eigen::Vector2i(x, y))];
          },
          [&](int x, double val) {
            distance_buffer_[grid_map_ptr_->getIndex(Eigen::Vector2i(x, y))] =
                grid_map_ptr_->getResolution() * std::sqrt(val);
          },
          0, grid_map_ptr_->getWidth() - 1, 0);
    }
    // std::cout << "fill x " << std::endl;
  }
  double getDistance(const Eigen::Vector2d &pos) {
    int index = grid_map_ptr_->getIndex(pos);
    return distance_buffer_[index];
  }
  
  void getESDFPointCloud(pcl::PointCloud<pcl::PointXYZI> &cloud, const std::string & frame_id) {
    cloud.clear();
    double dist;
    pcl::PointXYZI pt;
    const double min_dist = 0.0;
    const double max_dist = 3.0;
    for (int x = 0; x <= grid_map_ptr_->getWidth(); x++) {
      for (int y = 0; y <= grid_map_ptr_->getHeight(); y++) {
        Eigen::Vector2i eigen_index(x, y);
        auto pos = grid_map_ptr_->getCartesianCoordinate(eigen_index);
        dist = getDistance(pos);
        dist = std::min(dist, max_dist);
        dist = std::max(dist, min_dist);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z =-0.1;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
        if (dist < 0) {
          std::cout << "dist is " << dist << std::endl;
        }
      }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = frame_id;
  }
  bool isInMap(const Eigen::Vector2d &pos) {
    Eigen::Vector2i eigein_index = grid_map_ptr_->getGridMapIndex(pos);
    return grid_map_ptr_->isVerify(eigein_index);
  }

  bool isInMap(const Eigen::Vector2i &idx) {
    return grid_map_ptr_->isVerify(idx);
  }
  void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double &dist,
                           Eigen::Vector2d &grad) {
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    getSurroundPts(pos, sur_pts, diff);
    double dists[2][2];
    // 获取最近点与最近障碍物体的距离
    getSurroundDistance(sur_pts, dists);
    interpolateBilinear(dists, diff, dist, grad);
  }
  void interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff,
                           double &value, Eigen::Vector2d &grad) {
    double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];
    double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];
    value = (1 - diff(1)) * v0 + diff(1) * v1;

    grad[1] = (v1 - v0) / grid_map_ptr_->getResolution();
    grad[0] = ((1 - diff[1]) * (values[1][0] - values[0][0]) +
               diff[1] * (values[1][1] - values[0][1])) /
              grid_map_ptr_->getResolution();
  }
  void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
                      Eigen::Vector2d &diff) {
    if (!isInMap(pos)) {
      std::cout << "pos invalid for interpolation." << std::endl;
    }
    /* interpolation position */
    Eigen::Vector2d pos_m =
        pos - 0.5 * grid_map_ptr_->getResolution() * Eigen::Vector2d::Ones();
    Eigen::Vector2i idx = grid_map_ptr_->getGridMapIndex(pos_m);
    Eigen::Vector2d idx_pos = grid_map_ptr_->getCartesianCoordinate(idx);
    diff = (pos - idx_pos) / grid_map_ptr_->getResolution();
    for (int x = 0; x < 2; x++) {
      for (int y = 0; y < 2; y++) {
        Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
        Eigen::Vector2d current_pos =
            grid_map_ptr_->getCartesianCoordinate(current_idx);
        pts[x][y] = current_pos;
      }
    }
  }
  void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
    for (int x = 0; x < 2; x++) {
      for (int y = 0; y < 2; y++) {
        dists[x][y] = getDistance(pts[x][y]);
      }
    }
  }

private:
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end,
                int dim) {
    int v[map_size_(dim)];
    double z[map_size_(dim) + 1];
    int k = start;
    v[start] = start;
    z[start] = -std::numeric_limits<double>::max();
    z[start + 1] = std::numeric_limits<double>::max();
    for (int q = start + 1; q <= end; q++) {
      k++;
      double s;
      do {
        k--;
        s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) /
            (2 * q - 2 * v[k]);
      } while (s <= z[k]);
      k++;
      v[k] = q;
      z[k] = s;
      z[k + 1] = std::numeric_limits<double>::max();
    }
    k = start;
    for (int q = start; q <= end; q++) {
      while (z[k + 1] < q)
        k++;
      double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
      f_set_val(q, val);
    }
  }
  

private:
  GridMap::Ptr grid_map_ptr_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_buffer_; //缓存y维度的距离
  Eigen::Vector2i map_size_;

public:
  typedef std::shared_ptr<ESDFMap> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace plan_environment
} // namespace motion_planning
#endif // MOTION_PLANNING_ESDF_MAP_H_