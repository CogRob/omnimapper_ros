#pragma once

#include <gtsam/geometry/Pose3.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace omnimapper {

/** Converts a tf to a gtsam::Pose3 */
gtsam::Pose3 tf2pose3(
    const geometry_msgs::msg::TransformStamped& transform_msg);

tf2::Transform pose3totf(gtsam::Pose3& pose);

}  // namespace omnimapper
