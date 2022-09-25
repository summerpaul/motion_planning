/**
 * @Author: Yunkai Xia
 * @Date:   2022-08-29 14:25:06
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-08-29 14:27:20
 */
#include "polynomial_traj_ros.h"

PolynomialTrajRos::PolynomialTrajRos() : pnh_("~") {}
bool PolynomialTrajRos::init() {
  pnh_.param<double>("max_vel", max_vel_, 1.5);
  pnh_.param<double>("max_acc", max_acc_, 2.0);
  click_point_sub_ = nh_.subscribe(
      "/initialpose", 1, &PolynomialTrajRos::rvizVehiclePoseCallback, this);
  poly_traj_path_vis_ = std::make_shared<RosVizTools>(nh_, "traj_path");
  return true;
}
void PolynomialTrajRos::rvizVehiclePoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  click_points_.push_back(msg->pose.pose);
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  points_x_.push_back(x);
  points_y_.push_back(y);
  int node_num = click_points_.size();
  if (node_num < 2) {
    return;
  }
  std::vector<Eigen::Vector2d> cubic_spine_points;
  auto cubic_spline = Spline2D(points_x_, points_y_);
  for (double i = 0; i < cubic_spline.s.back(); i += 0.1) {
    auto point = cubic_spline.calc_postion(i);
    cubic_spine_points.push_back(point);
  }
  Eigen::Vector2d start_pt, start_vel{0, 0}, start_acc{0, 0}, local_target_pt,
      local_target_vel{0, 0};
  start_pt[0] = click_points_[0].position.x;
  start_pt[1] = click_points_[0].position.y;
  local_target_pt[0] = click_points_[node_num - 1].position.x;
  local_target_pt[1] = click_points_[node_num - 1].position.y;
  double dist = (start_pt - local_target_pt).norm();
  //  std::cout << "dist is " << dist << std::endl;
  double time = pow(max_vel_, 2) / max_acc_ > dist
                    ? sqrt(dist / max_acc_)
                    : (dist - pow(max_vel_, 2) / max_acc_) / max_vel_ +
                          2 * max_vel_ / max_acc_;
  //  std::cout << "time is " <<time << std::endl;
  if (click_points_.size() == 2) {
    double ts = 0.01;
    //    std::cout << "ts is " << ts << std::endl;
    poly_traj_ = PolynomialTraj::one_segment_traj_gen(
        start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
        Eigen::Vector2d::Zero(), time);
    std::cout << "poly_traj_ finish " << std::endl;

    //    std::cout << "time_sum is " << time_sum << std::endl;
    std::vector<Eigen::Vector2d> segment_point;
    for (double t = 0; t < time; t += ts) {
      auto point = poly_traj_.evaluate(t);
      segment_point.push_back(point);
    }
    //    std::cout << "publish path " << std::endl;
    polyTrajPathPublish(segment_point, cubic_spine_points);
    return;
  }

  Eigen::MatrixXd pos(2, node_num);
  Eigen::Vector2d vec_point;
  Eigen::VectorXd t =
      Eigen::VectorXd::Ones(node_num - 1) * time / (node_num - 1);
  for (int i = 0; i < node_num; i++) {
    vec_point[0] = click_points_[i].position.x;
    vec_point[1] = click_points_[i].position.y;
    pos.col(i) = vec_point;
  }

  //  std::cout << "pos" << pos << std::endl;
  //  std::cout << "t is " << t << std::endl;
  poly_traj_ = PolynomialTraj::minSnapTraj(
      pos, start_vel, local_target_vel, start_acc, Eigen::Vector2d::Zero(), t);
  double ts = 0.1;
  std::vector<Eigen::Vector2d> segment_point;
  for (double t = 0; t < time; t += ts) {
    auto point = poly_traj_.evaluate(t);
    segment_point.push_back(point);
  }
  std::cout << "publish path " << std::endl;
  polyTrajPathPublish(segment_point, cubic_spine_points);
  return;
}

void PolynomialTrajRos::polyTrajPathPublish(
    const std::vector<Eigen::Vector2d>& path,
    const std::vector<Eigen::Vector2d>& cubic_spline_points) {
  poly_traj_path_vis_->clear();
  std::string ns, frame_id("map");
  geometry_msgs::Point p;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 scale;
  ns = "points";
  visualization_msgs::Marker marker_nodelist =
      RosVizTools::newSphereList(0.1, ns, 0, ros_viz_tools::RED, frame_id);
  for (auto point : click_points_) {
    p.x = point.position.x;
    p.y = point.position.y;
    p.z = 0;
    marker_nodelist.points.push_back(p);
  }
  poly_traj_path_vis_->append(marker_nodelist);
  ns = "traj";
  visualization_msgs::Marker marker_linestrip =
      RosVizTools::newLineStrip(0.02, ns, 0, ros_viz_tools::GREEN, frame_id);
  for (auto point : path) {
    p.x = point[0];
    p.y = point[1];
    marker_linestrip.points.push_back(p);
  }
  poly_traj_path_vis_->append(marker_linestrip);

  ns = "cubic_spline";
  visualization_msgs::Marker marker_cubic_splinestrip =
      RosVizTools::newLineStrip(0.02, ns, 0, ros_viz_tools::BLUE, frame_id);
  for (auto point : cubic_spline_points) {
    p.x = point[0];
    p.y = point[1];
    marker_cubic_splinestrip.points.push_back(p);
  }
  poly_traj_path_vis_->append(marker_cubic_splinestrip);

  poly_traj_path_vis_->publish();
}
