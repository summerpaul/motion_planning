/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-26 09:54:11
 */
#ifndef MOTION_PLANNING_SPLINE_H_
#define MOTION_PLANNING_SPLINE_H_
#include <Eigen/Eigen>
#include <vector>
namespace motion_planning {
namespace common {

using Vec_d = std::vector<double>;

inline Vec_d vec_diff(Vec_d input) {
  Vec_d output;
  for (unsigned int i = 1; i < input.size(); i++) {
    output.push_back(input[i] - input[i - 1]);
  }
  return output;
};

inline Vec_d cum_sum(Vec_d input) {
  Vec_d output;
  double temp = 0;
  for (unsigned int i = 0; i < input.size(); i++) {
    temp += input[i];
    output.push_back(temp);
  }
  return output;
};

class Spline {
 public:
  Vec_d x;
  Vec_d y;
  int nx;
  Vec_d h;
  Vec_d a;
  Vec_d b;
  Vec_d c;
  // Eigen::VectorXd c;
  Vec_d d;

  Spline(){};
  // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
  Spline(Vec_d x_, Vec_d y_)
      : x(x_), y(y_), nx(x_.size()), h(vec_diff(x_)), a(y_) {
    Eigen::MatrixXd A = calc_A();
    Eigen::VectorXd B = calc_B();
    Eigen::VectorXd c_eigen = A.colPivHouseholderQr().solve(B);
    double* c_pointer = c_eigen.data();
    // Eigen::Map<Eigen::VectorXd>(c, c_eigen.rows(), 1) = c_eigen;
    c.assign(c_pointer, c_pointer + c_eigen.rows());

    for (int i = 0; i < nx - 1; i++) {
      d.push_back((c[i + 1] - c[i]) / (3.0 * h[i]));
      b.push_back((a[i + 1] - a[i]) / h[i] -
                  h[i] * (c[i + 1] + 2 * c[i]) / 3.0);
    }
  };

  double calc(double t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx +
           d[seg_id] * dx * dx * dx;
  };

  double calc_d(double t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx - 1);
    double dx = t - x[seg_id];
    return b[seg_id] + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
  }

  double calc_dd(double t) {
    if (t < x.front() || t > x.back()) {
      throw std::invalid_argument(
          "received value out of the pre-defined range");
    }
    int seg_id = bisect(t, 0, nx);
    double dx = t - x[seg_id];
    return 2 * c[seg_id] + 6 * d[seg_id] * dx;
  }

 private:
  Eigen::MatrixXd calc_A() {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
    A(0, 0) = 1;
    for (int i = 0; i < nx - 1; i++) {
      if (i != nx - 2) {
        A(i + 1, i + 1) = 2 * (h[i] + h[i + 1]);
      }
      A(i + 1, i) = h[i];
      A(i, i + 1) = h[i];
    }
    A(0, 1) = 0.0;
    A(nx - 1, nx - 2) = 0.0;
    A(nx - 1, nx - 1) = 1.0;
    return A;
  };
  Eigen::VectorXd calc_B() {
    Eigen::VectorXd B = Eigen::VectorXd::Zero(nx);
    for (int i = 0; i < nx - 2; i++) {
      B(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] -
                 3.0 * (a[i + 1] - a[i]) / h[i];
    }
    return B;
  };

  int bisect(double t, int start, int end) {
    int mid = (start + end) / 2;
    if (t == x[mid] || end - start <= 1) {
      return mid;
    } else if (t > x[mid]) {
      return bisect(t, mid, end);
    } else {
      return bisect(t, start, mid);
    }
  }
};

class Spline2D {
 public:
  Spline sx;
  Spline sy;
  Vec_d s;
  Spline2D(){}

  Spline2D(Vec_d x, Vec_d y) {
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
  };


  Eigen::Vector2d calc_postion(double s_t) {
    double x = sx.calc(s_t);
    double y = sy.calc(s_t);
    return Eigen::Vector2d(x, y);
  };

  double calc_curvature(double s_t) {
    double dx = sx.calc_d(s_t);
    double ddx = sx.calc_dd(s_t);
    double dy = sy.calc_d(s_t);
    double ddy = sy.calc_dd(s_t);
    return (ddy * dx - ddx * dy) / (dx * dx + dy * dy);
  };

  double calc_yaw(double s_t) {
    double dx = sx.calc_d(s_t);
    double dy = sy.calc_d(s_t);
    return std::atan2(dy, dx);
  };

 private:
  Vec_d calc_s(Vec_d x, Vec_d y) {
    Vec_d ds;
    Vec_d out_s{0};
    Vec_d dx = vec_diff(x);
    Vec_d dy = vec_diff(y);

    for (unsigned int i = 0; i < dx.size(); i++) {
      ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    Vec_d cum_ds = cum_sum(ds);
    out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
    return out_s;
  };
};

}  // namespace common
}  // namespace motion_planning
#endif  // MOTION_PLANNING_SPLINE_H_