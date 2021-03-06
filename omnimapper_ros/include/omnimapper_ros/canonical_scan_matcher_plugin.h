#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/trigger.h>

#include <laser_geometry/laser_geometry.hpp>
#include <memory>

#include "omnimapper_ros/canonical_scan.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace omnimapper {
/** \brief Canonical Scan Matcher Plugin
 *
 * \author Carlos Nieto, Alex Trevor
 */
template <typename LScanT>
class CanonicalScanMatcherPlugin {
  using BetweenPose3Ptr =
      boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> >;
  using LaserScanPtr = boost::shared_ptr<sensor_msgs::msg::LaserScan>;
  using LaserScanPConstPtr =
      const boost::shared_ptr<sensor_msgs::msg::LaserScan>;
  using LScan = sensor_msgs::msg::LaserScan;

 public:
  CanonicalScanMatcherPlugin(omnimapper::OmniMapperBase* mapper,
                             std::shared_ptr<rclcpp::Node> ros_node,
                             std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  ~CanonicalScanMatcherPlugin();
  void laserScanCallback(const LaserScanPtr& lscan);
  void spin();
  bool spinOnce();
  bool getBaseToLaserTf(const std::string& frame_id);
  bool addConstraint(gtsam::Symbol sym1, gtsam::Symbol sym2,
                     scan_tools::CanonicalScan& cscan, bool always_add = true);
  bool tryLoopClosure(gtsam::Symbol sym);
  bool ready();
  // void setTriggeredMode (bool triggered_mode) { triggered_mode_ =
  // triggered_mode; } void trigger () { triggered_ = true; }
  void setTriggerFunctor(omnimapper::TriggerFunctorPtr ptr) { trigger_ = ptr; }
  LaserScanPConstPtr getLaserScanPtr(gtsam::Symbol sym);
  sensor_msgs::msg::PointCloud2 getPC2(gtsam::Symbol sym);

 protected:
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  OmniMapperBase* mapper_;
  laser_geometry::LaserProjection projector_;
  scan_tools::CanonicalScan seq_canonical_scan_;
  scan_tools::CanonicalScan lc_canonical_scan_;
  bool initialized_;
  std::map<gtsam::Symbol, LaserScanPtr> lscans_;
  std::map<gtsam::Symbol, gtsam::Point3> lscan_centroids_;
  std::map<gtsam::Symbol, sensor_msgs::msg::PointCloud2> clouds_;
  LaserScanPtr current_lscan_;
  bool have_new_lscan_;
  boost::mutex current_cloud_mutex_;
  boost::mutex current_lscan_mutex_;
  boost::condition_variable scan_cv_;
  bool first_;
  double trans_noise_;
  double rot_noise_;
  bool debug_;
  bool overwrite_timestamps_;
  float leaf_size_;
  float score_threshold_;
  bool downsample_;
  std::string base_frame_name_;
  gtsam::Symbol previous_sym_;
  gtsam::Symbol previous2_sym_;
  gtsam::Symbol previous3_sym_;
  bool add_identity_on_failure_;
  bool add_multiple_links_;
  bool add_loop_closures_;
  bool paused_;
  int count_;
  TriggerFunctorPtr trigger_;
  Time triggered_time_;
  tf2::Transform base_to_laser_;
  tf2::Transform laser_to_base_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_marker_array_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_msg_pub1_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_scan_msg_pub2_;
};

}  // namespace omnimapper

gtsam::Pose3 doCSM_impl(const sensor_msgs::msg::LaserScan& from_scan,
                        const sensor_msgs::msg::LaserScan& to_scan,
                        const gtsam::Pose3& odometry_relative_pose,
                        bool& worked,
                        gtsam::noiseModel::Gaussian::shared_ptr& noise_model,
                        scan_tools::CanonicalScan& canonicalScan,
                        tf2::Transform& base_to_laser_tf,
                        bool laser_mode = true);

sensor_msgs::msg::LaserScan SmoothScan(
    const sensor_msgs::msg::LaserScan& msg_in);

gtsam::Point3 GetMeanLaserPoint(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud);
