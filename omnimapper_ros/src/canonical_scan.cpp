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

#include <omnimapper_ros/canonical_scan.h>

namespace scan_tools {

CanonicalScan::CanonicalScan(std::shared_ptr<rclcpp::Node> ros_node)
    : ros_node_(ros_node) {
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;
  // sm_debug_write(1);
}

void CanonicalScan::initParams() {
  // **** input type - laser scan, or point clouds?
  // if false, will subscribe to LaserScan msgs on /scan.
  // if true, will subscribe to PointCloud2 msgs on /cloud
  input_.min_reading = 0.1;
  input_.max_reading = 10.0;
  input_.do_compute_covariance = 1;
  if (!ros_node_->get_parameter("use_cloud_input", use_cloud_input_))
    use_cloud_input_ = true;

  if (use_cloud_input_) {
    if (!ros_node_->get_parameter("cloud_range_min", cloud_range_min_))
      cloud_range_min_ = 0.1;
    if (!ros_node_->get_parameter("cloud_range_max", cloud_range_max_))
      cloud_range_max_ = 50.0;
    input_.min_reading = cloud_range_min_;
    input_.max_reading = cloud_range_max_;
  }

  // **** What predictions are available to speed up the ICP?
  // 1) imu - [theta] from imu yaw angle - /odom topic
  // 2) odom - [x, y, theta] from wheel odometry - /imu topic
  // 3) alpha_beta - [x, y, theta] from simple tracking filter - no topic req.
  // If more than one is enabled, priority is imu > odom > alpha_beta

  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!ros_node_->get_parameter("max_angular_correction_deg",
                                input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!ros_node_->get_parameter("max_linear_correction",
                                input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!ros_node_->get_parameter("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!ros_node_->get_parameter("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!ros_node_->get_parameter("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!ros_node_->get_parameter("max_correspondence_dist",
                                input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!ros_node_->get_parameter("sigma", input_.sigma)) input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!ros_node_->get_parameter("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!ros_node_->get_parameter("restart", input_.restart)) input_.restart = 0;

  // Restart: Threshold for restarting
  if (!ros_node_->get_parameter("restart_threshold_mean_error",
                                input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!ros_node_->get_parameter("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!ros_node_->get_parameter("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!ros_node_->get_parameter("clustering_threshold",
                                input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!ros_node_->get_parameter("orientation_neighbourhood",
                                input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!ros_node_->get_parameter("use_point_to_line_distance",
                                input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 0;

  // Discard correspondences based on the angles
  if (!ros_node_->get_parameter("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!ros_node_->get_parameter("do_alpha_test_thresholdDeg",
                                input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!ros_node_->get_parameter("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!ros_node_->get_parameter("outliers_adaptive_order",
                                input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!ros_node_->get_parameter("outliers_adaptive_mult",
                                input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar
  // angle of the points of one scan in the new position. If the polar
  // angle is
  // not a monotone function of the readings index, it means that the
  // surface is not visible in the next position. If it is not visible,
  // then we don't use it for matching.
  if (!ros_node_->get_parameter("do_visibility_test",
                                input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!ros_node_->get_parameter("outliers_remove_doubles",
                                input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method
  // http://purl.org/censi/2006/icpcov
  if (!ros_node_->get_parameter("do_compute_covariance",
                                input_.do_compute_covariance))
    input_.do_compute_covariance = 1;

  // Checks that find_correspondences_tricks gives the right answer
  if (!ros_node_->get_parameter("debug_verify_tricks",
                                input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to
  // compute the incidence beta, and the factor (1/cos^2(beta)) used to weight
  // the correspondence.");
  if (!ros_node_->get_parameter("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!ros_node_->get_parameter("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;
}
gtsam::Matrix upgrade_pose2_pose3_cov(const gtsam::Matrix& dp2) {
  gtsam::Matrix mat = gtsam::zeros(6, 6);
  for (unsigned int i = 0; i < 6; i++) {
    for (unsigned int j = 0; j < 6; j++) {
      if (i == j)
        mat(i, j) = 0.1;
      else
        mat(i, j) = 0.0;
    }
  }
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      mat(i + 2, j + 2) = dp2(i, j);
    }
  }
  return mat;
}
gtsam::Matrix gsl_to_gtsam_matrix(const gsl_matrix* inp, bool& valid) {
  valid = true;
  if (!inp) {
    printf("No matrix in inp!\n");
    assert(false);
  }
  gtsam::Matrix mat = gtsam::zeros(inp->size1, inp->size2);
  for (unsigned int i = 0; i < inp->size1; i++) {
    for (unsigned int j = 0; j < inp->size2; j++) {
      mat(i, j) = gsl_matrix_get(inp, i, j);
      if (isnan(mat(i, j))) valid = false;
      if (isinf(mat(i, j))) valid = false;
    }
  }
  return mat;
}
bool CanonicalScan::processScan(
    LDP& curr_ldp_scan, LDP& prev_ldp_scan,
    const gtsam::Pose2& initial_rel_pose,
    // const gtsam::Pose3& initial_rel_pose,
    // gtsam::Pose3& output_rel_pose,
    gtsam::Pose2& output_rel_pose,
    gtsam::noiseModel::Gaussian::shared_ptr& icp_cov) {
  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of 0
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  prev_ldp_scan->odometry[0] = 0;
  prev_ldp_scan->odometry[1] = 0;
  prev_ldp_scan->odometry[2] = 0;

  prev_ldp_scan->estimate[0] = 0;
  prev_ldp_scan->estimate[1] = 0;
  prev_ldp_scan->estimate[2] = 0;

  prev_ldp_scan->true_pose[0] = 0;
  prev_ldp_scan->true_pose[1] = 0;
  prev_ldp_scan->true_pose[2] = 0;

  input_.laser_ref = curr_ldp_scan;   // curr_ldp_scan;
  input_.laser_sens = prev_ldp_scan;  // prev_ldp_scan;

  // **** estimated change since last scan

  input_.first_guess[0] = initial_rel_pose.x();
  input_.first_guess[1] = initial_rel_pose.y();
  input_.first_guess[2] = initial_rel_pose.theta();

  // *** scan match - using icp (xy means x and y are already computed)
  // sm_icp_xy(&input_, &output_);
  sm_icp(&input_, &output_);

  if (output_.valid) {
    // the correction of the laser's position, in the laser frame

    // printf("%f, %f, %f\n", output_.x[0], output_.x[1], output_.x[2]);

    // output_rel_pose = gtsam::Pose2(output_.x[2],output_.x[0], output_.x[1]);
    output_rel_pose = gtsam::Pose2(output_.x[0], output_.x[1], output_.x[2]);
    bool valid = true;
    icp_cov = gtsam::noiseModel::Gaussian::Covariance(
        upgrade_pose2_pose3_cov(gsl_to_gtsam_matrix(output_.cov_x_m, valid)));
    if (!valid) return false;

  } else {
    RCLCPP_WARN(ros_node_->get_logger(), "Error in scan matching");
    output_rel_pose = gtsam::Pose2(0.0, 0.0, 0.0);
  }

  return output_.valid;
}

void CanonicalScan::laserScanToLDP(const sensor_msgs::msg::LaserScan& scan_msg,
                                   LDP& ldp) {
  unsigned int n = scan_msg.ranges.size();
  ldp = ld_alloc_new(n);

  for (unsigned int i = 0; i < n; i++) {
    // calculate position in laser frame

    double r = scan_msg.ranges[i];

    if (r > scan_msg.range_min && r < scan_msg.range_max) {
      // fill in laser scan data

      ldp->valid[i] = 1;
      ldp->readings[i] = r;
    } else {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;  // for invalid range
    }

    ldp->theta[i] = scan_msg.angle_min + i * scan_msg.angle_increment;

    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}

void CanonicalScan::PointCloudToLDP(const sensor_msgs::msg::PointCloud& cloud,
                                    const sensor_msgs::msg::LaserScan& scan,
                                    LDP& ldp) {
  unsigned int n =
      scan.ranges.size();  // cloud.points.size();//width * cloud.height ;
  ldp = ld_alloc_new(n);
  if (n > 0) {
    double min_cloud_angle_ = scan.angle_min;
    unsigned int j = 0;
    for (unsigned int i = 0; i < n; i++) {
      // calculate position in laser frame

      double r = scan.ranges[i];

      if (r > scan.range_min && r < scan.range_max) {
        ldp->valid[i] = 1;
        ldp->points[i].p[0] = cloud.points[j].x;
        ldp->points[i].p[1] = cloud.points[j].y;

        // these are fake, but csm complains if left empty
        ldp->readings[i] = 1.0;  // r;
        j++;
      } else {
        ldp->valid[i] = 0;
        // these are fake, but csm complains if left empty
        ldp->readings[i] = -1;  // for invalid range
      }
      // these are fake, but csm complains if left empty
      ldp->theta[i] = min_cloud_angle_ + (double)i * scan.angle_increment;

      ldp->cluster[i] = -1;
    }
    assert(j == cloud.points.size());

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];
  }

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}
}  // namespace scan_tools
