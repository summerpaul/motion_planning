/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-23 14:32:39
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-23 14:40:22
 */
#include <stdint.h>

#ifndef __TIME_H__
#define __TIME_H__
#include <chrono>
#include <ostream>
#include <ratio>
// 参考cartographter中的时间戳
namespace motion_planning {
namespace common {
struct UniversalTimeScaleClock {
  using rep = int64_t;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};
// 表示通用时间标度的持续时间和时间戳, 它们是64位整数,
// 表示自世界标准时间1月1日1月1日开始的纪元以来的100纳秒刻度.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

Duration fromSeconds(double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}

Duration fromMilliseconds(int64_t milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

double toSeconds(Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}
Time fromUniversal(int64_t ticks) { return Time(Duration(ticks)); }
// 转换时间戳的单位
int64_t toUniversal(Time time) { return time.time_since_epoch().count(); }
// 打印输出
std::ostream &operator<<(std::ostream &os, Time time) {
  os << std::to_string(toUniversal(time));
  return os;
}
} // namespace common
} // namespace motion_planning

#endif /* __TIME_H__ */
