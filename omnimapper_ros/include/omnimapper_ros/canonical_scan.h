/** \note The functions in this file are modified versions of the scan_tools
 * package from CCNY, by William Morris and Ivan Dryanovski, modifications by
 * John G. Rogers III. See scan_tools/laser_scan_matcher/ for the original
 * functions.
 *
 * \note The techniques implemented within are based the following, please cite
 * if used in academic work: A. Censi, "An ICP variant using a point-to-line
 * metric". Proceedings of the IEEE International Conference on Robotics and
 * Automation (ICRA), 2008
 */

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>

#include <memory>

#include "omnimapper_ros/csm_math_functions.h"
#include "omnimapper_ros/include_csm.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

namespace scan_tools {

class CanonicalScan {
 protected:
  std::shared_ptr<rclcpp::Node> ros_node_;

  sm_params input_;
  sm_result output_;
  double cloud_range_min_;
  double cloud_range_max_;
  bool use_cloud_input_;

 public:
  explicit CanonicalScan(std::shared_ptr<rclcpp::Node> ros_node);
  void initParams();
  bool processScan(LDP& curr_ldp_scan, LDP& prev_ldp_scan,
                   const gtsam::Pose2& initial_rel_pose,
                   gtsam::Pose2& output_rel_pose,
                   gtsam::noiseModel::Gaussian::shared_ptr& icp_cov);
  void PointCloudToLDP(const sensor_msgs::msg::PointCloud& cloud,
                       const sensor_msgs::msg::LaserScan& scan, LDP& ldp);
  void laserScanToLDP(const sensor_msgs::msg::LaserScan& scan_msg, LDP& ldp);
};

}  // namespace scan_tools
