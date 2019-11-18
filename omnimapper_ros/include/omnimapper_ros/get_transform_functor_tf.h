#pragma once

#include <omnimapper/get_transform_functor.h>

#include "omnimapper_ros/ros_tf_utils.h"
#include "tf2_ros/buffer.h"

namespace omnimapper {
/** \brief GetTransformFunctorTF enables lookup of a (potentially  dynamic)
 * transform at a given time, using ROS's TF. See
 * omnimapper/get_transform_functor.h for more details.
 */
class GetTransformFunctorTF : public GetTransformFunctor {
 public:
  GetTransformFunctorTF(const std::string& sensor_frame_name,
                        const std::string& base_frame_name,
                        std::shared_ptr<rclcpp::Node> ros_node,
                        std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  Eigen::Affine3d operator()(omnimapper::Time t);

 protected:
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string sensor_frame_name_;
  std::string base_frame_name_;
};

}  // namespace omnimapper
