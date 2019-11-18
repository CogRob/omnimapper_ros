#pragma once

#include <gtsam/geometry/Pose3.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"

namespace omnimapper {

/** Converts a tf to a gtsam::Pose3 */
gtsam::Pose3 tf2pose3(const geometry_msgs::msg::TransformStamped& transform);

tf2::Transform pose3totf(gtsam::Pose3& pose);

}  // namespace omnimapper
