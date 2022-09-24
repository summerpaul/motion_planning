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
