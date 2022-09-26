/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-26 09:54:09
 */
#ifndef MOTION_PLANNING_POLYNOMIAL_TRAJ_H_
#define MOTION_PLANNING_POLYNOMIAL_TRAJ_H_
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
// 参考faster planner的多项式
namespace motion_planning {
namespace common {
using std::vector;
class PolynomialTraj {
public:
  PolynomialTraj() {}
  ~PolynomialTraj() {}
  void init() {
    num_seg = times.size();
    time_sum = 0.0;
    for (int i = 0; i < times.size(); ++i) {
      time_sum += times[i];
    }
  }
  void reset() {
    times.clear(), cxs.clear(), cys.clear();
    time_sum = 0.0, num_seg = 0;
  }
  void addSegment(vector<double> cx, vector<double> cy, double t) {
    cxs.push_back(cx), cys.push_back(cy), times.push_back(t);
  }
  vector<double> getTimes() { return times; }

  vector<vector<double>> getCoef(int axis) {
    switch (axis) {
    case 0:
      return cxs;
    case 1:
      return cys;
    default:
      std::cout << "\033[31mIllegal axis!\033[0m" << std::endl;
    }

    vector<vector<double>> empty;
    return empty;
  }

  Eigen::Vector2d evaluate(double t) {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }
    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd cx(order), cy(order), tv(order);
    for (int i = 0; i < order; ++i) {
      cx(i) = cxs[idx][i], cy(i) = cys[idx][i];
      tv(order - 1 - i) = std::pow(t, double(i));
    }
    Eigen::Vector2d pt;
    pt(0) = tv.dot(cx), pt(1) = tv.dot(cy);
    return pt;
  }

  Eigen::Vector2d evaluateVel(double t) {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }

    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd vx(order - 1), vy(order - 1);

    /* coef of vel */
    for (int i = 0; i < order - 1; ++i) {
      vx(i) = double(i + 1) * cxs[idx][order - 2 - i];
      vy(i) = double(i + 1) * cys[idx][order - 2 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 1);
    for (int i = 0; i < order - 1; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector2d vel;
    vel(0) = tv.dot(vx), vel(1) = tv.dot(vy);
    return vel;
  }

  Eigen::Vector2d evaluateAcc(double t) {
    /* detetrmine segment num */
    int idx = 0;
    while (times[idx] + 1e-4 < t) {
      t -= times[idx];
      ++idx;
    }
    /* evaluation */
    int order = cxs[idx].size();
    Eigen::VectorXd ax(order - 2), ay(order - 2);

    /* coef of vel */
    for (int i = 0; i < order - 2; ++i) {
      ax(i) = double((i + 2) * (i + 1)) * cxs[idx][order - 3 - i];
      ay(i) = double((i + 2) * (i + 1)) * cys[idx][order - 3 - i];
    }
    double ts = t;
    Eigen::VectorXd tv(order - 2);
    for (int i = 0; i < order - 2; ++i)
      tv(i) = pow(ts, i);

    Eigen::Vector2d acc;
    acc(0) = tv.dot(ax), acc(1) = tv.dot(ay);
    return acc;
  }

  double getTimeSum() { return this->time_sum; }

  vector<Eigen::Vector2d> getTraj() {
    double eval_t = 0.0;
    traj_vec2d.clear();
    while (eval_t < time_sum) {
      Eigen::Vector2d pt = evaluate(eval_t);
      traj_vec2d.push_back(pt);
      eval_t += 0.01;
    }
    return traj_vec2d;
  }

  double getLength() {
    length = 0.0;
    Eigen::Vector2d p_l = traj_vec2d[0], p_n;
    for (int i = 1; i < traj_vec2d.size(); ++i) {
      p_n = traj_vec2d[i];
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double getMeanVel() { return length / time_sum; }

  double getAccCost() {
    double cost = 0.0;
    int order = cxs[0].size();

    for (int s = 0; s < times.size(); ++s) {
      Eigen::Vector2d um;
      um(0) = 2 * cxs[s][order - 3], um(1) = 2 * cys[s][order - 3];
      cost += um.squaredNorm() * times[s];
    }

    return cost;
  }

  double getJerk() {
    double jerk = 0.0;

    /* evaluate jerk */
    for (int s = 0; s < times.size(); ++s) {
      Eigen::VectorXd cxv(cxs[s].size()), cyv(cys[s].size());
      /* convert coefficient */
      int order = cxs[s].size();
      for (int j = 0; j < order; ++j) {
        cxv(j) = cxs[s][order - 1 - j], cyv(j) = cys[s][order - 1 - j];
      }
      double ts = times[s];

      /* jerk matrix */
      Eigen::MatrixXd mat_jerk(order, order);
      mat_jerk.setZero();
      for (double i = 3; i < order; i += 1)
        for (double j = 3; j < order; j += 1) {
          mat_jerk(i, j) = i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) *
                           pow(ts, i + j - 5) / (i + j - 5);
        }

      jerk += (cxv.transpose() * mat_jerk * cxv)(0, 0);
      jerk += (cyv.transpose() * mat_jerk * cyv)(0, 0);
    }

    return jerk;
  }

  void getMeanAndMaxVel(double &mean_v, double &max_v) {
    int num = 0;
    mean_v = 0.0, max_v = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd vx(order - 1), vy(order - 1);

      /* coef of vel */
      for (int i = 0; i < order - 1; ++i) {
        vx(i) = double(i + 1) * cxs[s][order - 2 - i];
        vy(i) = double(i + 1) * cys[s][order - 2 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 1);
        for (int i = 0; i < order - 1; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d vel;
        vel(0) = tv.dot(vx), vel(1) = tv.dot(vy);
        double vn = vel.norm();
        mean_v += vn;
        if (vn > max_v)
          max_v = vn;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_v = mean_v / double(num);
  }

  void getMeanAndMaxAcc(double &mean_a, double &max_a) {
    int num = 0;
    mean_a = 0.0, max_a = -1.0;
    for (int s = 0; s < times.size(); ++s) {
      int order = cxs[s].size();
      Eigen::VectorXd ax(order - 2), ay(order - 2);

      /* coef of acc */
      for (int i = 0; i < order - 2; ++i) {
        ax(i) = double((i + 2) * (i + 1)) * cxs[s][order - 3 - i];
        ay(i) = double((i + 2) * (i + 1)) * cys[s][order - 3 - i];
      }
      double ts = times[s];

      double eval_t = 0.0;
      while (eval_t < ts) {
        Eigen::VectorXd tv(order - 2);
        for (int i = 0; i < order - 2; ++i)
          tv(i) = pow(ts, i);
        Eigen::Vector3d acc;
        acc(0) = tv.dot(ax), acc(1) = tv.dot(ay);
        double an = acc.norm();
        mean_a += an;
        if (an > max_a)
          max_a = an;
        ++num;

        eval_t += 0.01;
      }
    }

    mean_a = mean_a / double(num);
  }

  static PolynomialTraj
  minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector2d &start_vel,
              const Eigen::Vector2d &end_vel, const Eigen::Vector2d &start_acc,
              const Eigen::Vector2d &end_acc, const Eigen::VectorXd &Time) {
    int seg_num = Time.size();
    Eigen::MatrixXd poly_coeff(seg_num, 2 * 6);
    Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num);

    int num_f, num_p; // number of fixed and free variables
    int num_d;        // number of all segments' derivatives

    const static auto Factorial = [](int x) {
      int fac = 1;
      for (int i = x; i > 0; i--)
        fac = fac * i;
      return fac;
    };

    /* ---------- end point derivative ---------- */
    Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
    for (int k = 0; k < seg_num; k++) {
      /* position to derivative */
      Dx(k * 6) = Pos(0, k);
      Dx(k * 6 + 1) = Pos(0, k + 1);
      Dy(k * 6) = Pos(1, k);
      Dy(k * 6 + 1) = Pos(1, k + 1);

      if (k == 0) {
        Dx(k * 6 + 2) = start_vel(0);
        Dy(k * 6 + 2) = start_vel(1);

        Dx(k * 6 + 4) = start_acc(0);
        Dy(k * 6 + 4) = start_acc(1);
      } else if (k == seg_num - 1) {
        Dx(k * 6 + 3) = end_vel(0);
        Dy(k * 6 + 3) = end_vel(1);

        Dx(k * 6 + 5) = end_acc(0);
        Dy(k * 6 + 5) = end_acc(1);
      }
    }

    /* ---------- Mapping Matrix A ---------- */
    Eigen::MatrixXd Ab;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for (int k = 0; k < seg_num; k++) {
      Ab = Eigen::MatrixXd::Zero(6, 6);
      for (int i = 0; i < 3; i++) {
        Ab(2 * i, i) = Factorial(i);
        for (int j = i; j < 6; j++)
          Ab(2 * i + 1, j) =
              Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
      }
      A.block(k * 6, k * 6, 6, 6) = Ab;
    }

    /* ---------- Produce Selection Matrix C' ---------- */
    Eigen::MatrixXd Ct, C;

    num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
    num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
    num_d = 6 * seg_num;
    Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
    Ct(0, 0) = 1;
    Ct(2, 1) = 1;
    Ct(4, 2) = 1; // stack the start point
    Ct(1, 3) = 1;
    Ct(3, 2 * seg_num + 4) = 1;
    Ct(5, 2 * seg_num + 5) = 1;

    Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
    Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
    Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
    Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

    for (int j = 2; j < seg_num; j++) {
      Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
      Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
      Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
      Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
      Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
      Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
    }

    C = Ct.transpose();

    Eigen::VectorXd Dx1 = C * Dx;
    Eigen::VectorXd Dy1 = C * Dy;

    /* ---------- minimum snap matrix ---------- */
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for (int k = 0; k < seg_num; k++) {
      for (int i = 3; i < 6; i++) {
        for (int j = 3; j < 6; j++) {
          Q(k * 6 + i, k * 6 + j) = i * (i - 1) * (i - 2) * j * (j - 1) *
                                    (j - 2) / (i + j - 5) *
                                    pow(Time(k), (i + j - 5));
        }
      }
    }

    /* ---------- R matrix ---------- */
    Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

    Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4);

    Dxf = Dx1.segment(0, 2 * seg_num + 4);
    Dyf = Dy1.segment(0, 2 * seg_num + 4);

    Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
    Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
    Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
    Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

    Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
    Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
    Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
    Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2,
                  2 * seg_num - 2);

    /* ---------- close form solution ---------- */
    Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2);
    Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
    Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;

    Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
    Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;

    Px = (A.inverse() * Ct) * Dx1;
    Py = (A.inverse() * Ct) * Dy1;

    for (int i = 0; i < seg_num; i++) {
      poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
      poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    }

    /* ---------- use polynomials ---------- */
    PolynomialTraj poly_traj;
    for (int i = 0; i < poly_coeff.rows(); ++i) {
      vector<double> cx(6), cy(6);
      for (int j = 0; j < 6; ++j) {
        cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6);
      }
      reverse(cx.begin(), cx.end());
      reverse(cy.begin(), cy.end());
      double ts = Time(i);
      poly_traj.addSegment(cx, cy, ts);
    }

    return poly_traj;
  }

  static PolynomialTraj one_segment_traj_gen(const Eigen::Vector2d &start_pt,
                                             const Eigen::Vector2d &start_vel,
                                             const Eigen::Vector2d &start_acc,
                                             const Eigen::Vector2d &end_pt,
                                             const Eigen::Vector2d &end_vel,
                                             const Eigen::Vector2d &end_acc,
                                             double t) {
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
    Eigen::VectorXd Bx(6), By(6);
    C(0, 5) = 1;
    C(1, 4) = 1;
    C(2, 3) = 2;
    Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
    C.row(3) = Crow;
    Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
    C.row(4) = Crow;
    Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
    C.row(5) = Crow;

    Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0),
        end_acc(0);
    By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1),
        end_acc(1);

    Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
    Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);

    vector<double> cx(6), cy(6);
    for (int i = 0; i < 6; i++) {
      cx[i] = Cofx(i);
      cy[i] = Cofy(i);
    }

    PolynomialTraj poly_traj;
    poly_traj.addSegment(cx, cy, t);

    return poly_traj;
  }

private:
  vector<double> times;       //每段多项式的时间
  vector<vector<double>> cxs; // x轴多项式的系数
  vector<vector<double>> cys; // y轴多项式的系数
  double time_sum;
  int num_seg;
  vector<Eigen::Vector2d> traj_vec2d;
  double length;
};

} // namespace common
} // namespace motion_planning
#endif // MOTION_PLANNING_POLYNOMIAL_TRAJ_H_