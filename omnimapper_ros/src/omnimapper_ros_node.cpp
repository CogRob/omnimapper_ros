#include "omnimapper_ros/omnimapper_ros.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto g_node = rclcpp::Node::make_shared("OmniMapperROSNode", node_options);

  OmniMapperROS<pcl::PointXYZRGBA> omnimapper(g_node);

  rclcpp::spin(g_node);
  rclcpp::shutdown();

  g_node = nullptr;
  return 0;
}
