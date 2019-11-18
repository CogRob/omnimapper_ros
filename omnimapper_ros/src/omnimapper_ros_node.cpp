#include "omnimapper_ros/omnimapper_ros.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto g_node = rclcpp::Node::make_shared("OmniMapperROSNode");

  OmniMapperROS<pcl::PointXYZRGBA> omnimapper(g_node);

  rclcpp::spin(g_node);
  rclcpp::shutdown();

  g_node = nullptr;
  return 0;
}
