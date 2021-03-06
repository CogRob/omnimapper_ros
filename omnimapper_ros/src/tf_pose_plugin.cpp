#include <omnimapper_ros/tf_pose_plugin.h>

namespace omnimapper {

TFPosePlugin::TFPosePlugin(omnimapper::OmniMapperBase* mapper,
                           std::shared_ptr<rclcpp::Node> ros_node,
                           std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : mapper_(mapper),
      ros_node_(ros_node),
      tf_buffer_(tf_buffer),
      odom_frame_name_("odom"),
      base_frame_name_("camera_depth_optical_frame"),
      rotation_noise_(1.0),
      translation_noise_(1.0),
      debug_(false) {}

gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr TFPosePlugin::addRelativePose(
    boost::posix_time::ptime t1, gtsam::Symbol sym1,
    boost::posix_time::ptime t2, gtsam::Symbol sym2) {
  // Convert the timestamps
  rclcpp::Time rt1 = ptime2rostime(t1);
  rclcpp::Time rt2 = ptime2rostime(t2);

  // Get the poses
  geometry_msgs::msg::TransformStamped tf1;
  geometry_msgs::msg::TransformStamped tf2;

  // Add the factor
  bool got_tf = true;
  gtsam::Pose3 relative_pose = gtsam::Pose3::identity();
  try {
    tf1 = tf_buffer_->lookupTransform(odom_frame_name_, base_frame_name_,
                                      tf2_ros::fromMsg(rt1),
                                      tf2::durationFromSec(0.2));
    tf2 = tf_buffer_->lookupTransform(odom_frame_name_, base_frame_name_,
                                      tf2_ros::fromMsg(rt2),
                                      tf2::durationFromSec(0.2));
  } catch (tf2::TransformException ex) {
    RCLCPP_INFO(
        ros_node_->get_logger(),
        "OmniMapper reports :: Transform from %s to %s not yet available.  "
        "Exception: %s",
        odom_frame_name_.c_str(), base_frame_name_.c_str(), ex.what());
    RCLCPP_INFO(ros_node_->get_logger(), "Writing identity instead\n");
    got_tf = false;
  }

  if (got_tf) {
    gtsam::Pose3 pose1 = tf2pose3(tf1);
    gtsam::Pose3 pose2 = tf2pose3(tf2);
    // gtsam::Pose3 relative_pose = pose1.between (pose2);
    relative_pose = pose1.between(pose2);
  }

  if (debug_) {
    printf("TFPosePlugin: Adding factor between %zu and %zu\n", sym1.index(),
           sym2.index());
    printf("TFPosePlugin: Relative transform: %lf %lf %lf\n", relative_pose.x(),
           relative_pose.y(), relative_pose.z());
  }

  double trans_noise = translation_noise_;  // 1.0;

  gtsam::Vector noise_vector(6);
  noise_vector << roll_noise_,
    pitch_noise_,
    yaw_noise_,
    trans_noise,
    trans_noise,
    trans_noise;
  gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas(noise_vector);

  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between(
      new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose, noise));

  if (debug_) between->print("TF BetweenFactor:\n");

  return (between);
}

bool TFPosePlugin::ready() { return (true); }

}  // namespace omnimapper
