/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-23 14:42:37
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-23 18:32:46
 */
#include <stdint.h>

#ifndef MOTION_PLANNING_GRID_MAP_H_
#define MOTION_PLANNING_GRID_MAP_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

namespace motion_planning {
namespace plan_environment {

/***************** Mat转vector **********************/
template <typename _Tp>
std::vector<_Tp> convertMat2Vector(const cv::Mat &mat) {
  return (std::vector<_Tp>)(mat.reshape(1, 1));  //通道数不变，按行转为一行
}

/****************** vector转Mat *********************/
template <typename _Tp>
cv::Mat convertVector2Mat(std::vector<_Tp> v, int channels, int rows) {
  cv::Mat mat = cv::Mat(v);  //将vector变成单列的mat
  cv::Mat dest =
      mat.reshape(channels, rows).clone();  // PS：必须clone()一份，否则返回出错
  return dest;
}

class GridMap {
 public:
  // cv map转gridmap
  void createGridMap(const cv::Mat &mat, const double &root_x,
                     const double &root_y, const double &root_theta,
                     const double &resolution = 0.05,
                     const double &robot_radius = 0.3) {
    width_ = mat.cols;
    height_ = mat.rows;
    resolution_ = resolution;
    root_x_ = root_x;
    root_y_ = root_y;
    root_theta_ = root_theta;
    cv::Mat mat_fliped, dilated_image;
    int inf_step = ceil(robot_radius / resolution_);
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * inf_step + 1, 2 * inf_step + 1),
        cv::Point(inf_step, inf_step));
    cv::dilate(mat, dilated_image, element);
    cv::flip(dilated_image, mat_fliped, 0);
    data_ = convertMat2Vector<uchar>(mat_fliped);
  }
  //   点云转栅格地图
  void createGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                     const double &resolution = 0.05,
                     const double &robot_radius = 0.3) {
    resolution_ = resolution;
    double x_min{0}, x_max{0}, y_min{0}, y_max{0};
    for (std::size_t i = 0; i < cloud.points.size() - 1; i++) {
      if (i == 0) {
        x_min = x_max = cloud.points[i].x;
        y_min = y_max = cloud.points[i].y;
      }
      double x = cloud.points[i].x;
      double y = cloud.points[i].y;
      x_min = x < x_min ? x : x_min;
      x_max = x > x_max ? x : x_max;
      y_min = y < y_min ? y : y_min;
      y_max = y > y_max ? y : y_max;
    }
    root_x_ = x_min;
    root_y_ = y_min;
    root_theta_ = 0;
    width_ = int((x_max - x_min) / resolution_);
    height_ = int((y_max - y_min) / resolution_);
    map_size_2d_ = Eigen::Vector2d{x_max - x_min, y_max - y_min};
    map_size_2i_ = Eigen::Vector2i{width_, height_};
    origin_ = Eigen::Vector2d{root_x_, root_y_};
    // 创建栅格地图
    data_.resize(width_ * height_);
    data_.assign(width_ * height_, 0);
    // 膨胀地图
    int inf_step = ceil(robot_radius / resolution_);

    for (auto point : cloud.points) {
      int i = (point.x - root_x_) / resolution_;
      if (i < 0 || i > width_ - 1) continue;
      int j = (point.y - root_y_) / resolution_;
      if (j < 0 || j > height_ - 1) continue;
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y) {
          if (i + x < 0 || i + x > width_ - 1) continue;
          if (j + y < 0 || j + y > height_ - 1) continue;
          data_[i + x + (j + y) * width_] = 255;
        }
    }
  }
  // 转换为图像
  cv::Mat toImage() const {
    cv::Mat image = convertVector2Mat<uchar>(data_, 1, height_);
    cv::Mat image_fliped;
    cv::flip(image, image_fliped, 0);
    return image_fliped;
  };

  int getWidth() const { return width_; }

  int getHeight() const { return height_; }

  Eigen::Vector2d getMapSize2D() const { return map_size_2d_; }

  Eigen::Vector2i getMapSize2I() const { return map_size_2i_; }

  double getRootX() const { return root_x_; }

  double getRootY() const { return root_y_; }

  double getRootTheta() const { return root_theta_; }

  Eigen::Vector2d getOrigin() const { return origin_; }

  std::vector<uchar> data() const { return data_; }

  double getResolution() const { return resolution_; }

  void clear() {
    for (size_t i = 0; i < data_.size(); i++) {
      data_[i] = false;
    }
  }

  int getIndex(const Eigen::Vector2i &index) const {
    return index[0] + index[1] * width_;
  }

  void set(size_t index, uchar value) { data_[index] = value; }

  bool isOccupied(int index) const {
    if (data_[index] == 255) {
      return true;
    }
    return false;
  }

  bool isOccupied(const Eigen::Vector2i &index) const {
    return isOccupied(getIndex(index));
  }
  bool isVerify(const Eigen::Vector2i &index) const {
    if (index[0] >= 0 && index[0] < width_ && index[1] >= 0 &&
        index[1] < height_)
      return true;
    else
      return false;
  }
  // 需要进行坐标系变换，规划使用index的坐标系从0，0，0开始
  //   pose是从root_x, root_y, root_theta开始
  // 
  Eigen::Vector2i getGridMapIndex(const Eigen::Vector2d &pose) const {
    double x = (pose[0] - root_x_) * cos(root_theta_) +
               (pose[1] - root_y_) * sin(root_theta_);
    double y = -(pose[0] - root_x_) * sin(root_theta_) +
               (pose[1] - root_y_) * cos(root_theta_);
    return Eigen::Vector2i( int(x / resolution_),int(y / resolution_));
  }

  Eigen::Vector2d getCartesianCoordinate(const Eigen::Vector2i &index) const {
    double x = root_x_ + index[0] * resolution_ * cos(root_theta_) -
               index[1] * resolution_ * sin(root_theta_);
    double y = root_y_ + index[0] * resolution_ * sin(root_theta_) +
               index[1]* resolution_ * cos(root_theta_);

    return Eigen::Vector2d(x, y);
  }

 private:
  int width_;
  int height_;
  double resolution_;
  double root_x_;
  double root_y_;
  double root_theta_;
  std::vector<uchar> data_;
  Eigen::Vector2d map_size_2d_;
  Eigen::Vector2i map_size_2i_;
  Eigen::Vector2d origin_;

 public:
  typedef std::shared_ptr<GridMap> Ptr;
};

}  // namespace plan_environment
}  // namespace motion_planning
#endif /* __GRID_MAP_H__ */
