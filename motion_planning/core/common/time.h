/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-23 14:32:39
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-10-08 09:59:54
 */

#ifndef MOTION_PLANNING_TIME_H_
#define MOTION_PLANNING_TIME_H_
#include <ostream>
#include <sys/time.h>
namespace motion_planning::common {
static double getTimeNow() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

} // namespace motion_planning::common

#endif /* __TIME_H__ */
