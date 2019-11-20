#include "omnimapper_ros/get_transform_functor_tf.h"

#include <chrono>
#include <string>

#include "omnimapper_ros/ros_time_utils.h"
#include "rclcpp/rclcpp.hpp"

omnimapper::GetTransformFunctorTF::GetTransformFunctorTF(
    const std::string& sensor_frame_name, const std::string& base_frame_name,
    std::shared_ptr<rclcpp::Node> ros_node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : ros_node_(ros_node),
      tf_buffer_(tf_buffer),
      sensor_frame_name_(sensor_frame_name),
      base_frame_name_(base_frame_name) {
  bool ready = false;
  std::string message = "First attempt.";
  tf2::TimePoint time_to_lookup = tf2_ros::fromMsg(ros_node_->now());
  while (!ready && rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(ros_node_->get_logger(),
                "GetTransformFunctorTF: waiting for %s to %s to be ready: %s\n",
                sensor_frame_name.c_str(), base_frame_name.c_str(),
                message.c_str());
    ready = tf_buffer_->canTransform(base_frame_name_, sensor_frame_name_,
                                     time_to_lookup, &message);
    time_to_lookup = tf2_ros::fromMsg(ros_node_->now());
  }
}

Eigen::Affine3d omnimapper::GetTransformFunctorTF::operator()(
    omnimapper::Time t) {
  rclcpp::Time ros_time = ptime2rostime(t);
  geometry_msgs::msg::TransformStamped tf_transform;
  try {
    tf_transform = tf_buffer_->lookupTransform(
        base_frame_name_, sensor_frame_name_, tf2_ros::fromMsg(ros_time),
        tf2::durationFromSec(0.2));
  } catch (tf2::TransformException ex) {
    // printf ("GetTransformFunctorTF: Error looking up tf: %s\n", ex.what ());
    RCLCPP_ERROR(ros_node_->get_logger(),
                 "GetTransformFunctorTF: Error looking up tf: %s\n", ex.what());
    return (Eigen::Affine3d::Identity());
  }

  // TODO: make a function for this
  gtsam::Pose3 transform_p3 = omnimapper::tf2pose3(tf_transform);
  Eigen::Affine3d transform(transform_p3.matrix().cast<double>());
  return (transform);
}
