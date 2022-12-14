/**
 * @Author: Yunkai Xia
 * @Date:   2022-09-26 08:52:50
 * @Last Modified by:   Yunkai Xia
 * @Last Modified time: 2022-09-26 09:54:44
 */
#include "esdf_map_ros.h"

int main(int argc, char **argv) {
  /* code */
  ros::init(argc, argv, "esdf_map_test");
  ESDFMapRos esdf_map;
  if (!esdf_map.init()) {
    return 1;
  }
  ros::spin();

  return 0;
}
