/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-23 14:12:42
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-23 15:49:34
 */

#ifndef MOTION_PLANNING_MATH_H_
#define MOTION_PLANNING_MATH_H_
#include <cmath>
// 基本数学工具
namespace motion_planning {
namespace common {
// 角度转弧度
constexpr double degToRad(double deg) { return M_PI * deg / 180.; }
// 弧度转角度
constexpr double radToDeg(double rad) { return 180. * rad / M_PI; }

// 角度归一化 [-180, 180]
template <typename T> T normalizeAngleDeg(T deg) {
  const T degPi = T(180.0);
  while (deg > degPi) {
    deg -= 2. * degPi;
  }
  while (deg < -degPi) {
    deg += 2. * degPi;
  }
  return deg;
}
// 弧度归一化 [-pi, pi]
template <typename T> T normalizeAngleRad(T rad) {
  const T radPi = T(M_PI);
  while (rad > radPi) {
    rad -= 2. * radPi;
  }
  while (rad < -radPi) {
    rad += 2. * radPi;
  }
  return rad;
}
// 判断数学的正负
static int sign(double num) {
  if (num < 0)
    return -1;
  else if (num > 0)
    return 1;
  else
    return 0;
}
// 二维坐标变换

} // namespace common
} // namespace motion_planning

#endif /* __MATH_H__ */
