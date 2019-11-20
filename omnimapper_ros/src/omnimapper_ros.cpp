/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "omnimapper_ros/omnimapper_ros.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

template <typename PointT>
OmniMapperROS<PointT>::OmniMapperROS(std::shared_ptr<rclcpp::Node> ros_node)
    : ros_node_(ros_node),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(ros_node->get_clock(),
                                                   tf2::durationFromSec(500))),
      tf_listener_(*tf_buffer_),
      tf_broadcaster_(ros_node_),
      omb_(),
      tf_plugin_(&omb_, ros_node_, tf_buffer_),
      no_motion_plugin_(&omb_),
      icp_plugin_(&omb_),
      edge_icp_plugin_(&omb_),
      csm_plugin_(&omb_, ros_node_, tf_buffer_),
      vis_plugin_(&omb_, ros_node_, tf_buffer_),
      csm_vis_plugin_(&omb_, ros_node_, tf_buffer_),
      organized_segmentation_() {
  if (debug_)
    RCLCPP_INFO(ros_node_->get_logger(),
                "OmniMapperROS: Constructing... Loading ROS Params...");
  loadROSParams();

  // Use ROS Time instead of system clock
  omnimapper::GetTimeFunctorPtr time_functor_ptr(
      new omnimapper::GetROSTimeFunctor(ros_node_));
  omb_.setTimeFunctor(time_functor_ptr);
  omb_.setSuppressCommitWindow(suppress_commit_window_);

  // Optionally specify an alternate initial pose
  if (use_init_pose_) {
    gtsam::Pose3 init_pose(
        gtsam::Rot3::quaternion(init_qw_, init_qx_, init_qy_, init_qz_),
        gtsam::Point3(init_x_, init_y_, init_z_));
    omb_.setInitialPose(init_pose);
  }

  // Optionally get initial pose from TF
  if (init_pose_from_tf_) {
    bool got_tf = false;
    geometry_msgs::msg::TransformStamped init_transform;

    while (!got_tf) {
      got_tf = true;
      try {
        RCLCPP_INFO(ros_node_->get_logger(),
                    "Waiting for initial pose from %s to %s",
                    odom_frame_name_.c_str(), base_frame_name_.c_str());
        rclcpp::Time current_time = ros_node_->now();
        init_transform = tf_buffer_->lookupTransform(
            odom_frame_name_, base_frame_name_, tf2_ros::fromMsg(current_time),
            tf2::durationFromSec(1.0));
      } catch (tf2::TransformException ex) {
        RCLCPP_INFO(ros_node_->get_logger(), "Transform not yet available!");
        got_tf = false;
      }
    }

    gtsam::Pose3 init_pose = omnimapper::tf2pose3(init_transform);
    gtsam::Pose3 init_pose_inv = init_pose.inverse();
    omb_.setInitialPose(init_pose);
  }

  // Add the TF Pose Plugin
  tf_plugin_.setOdomFrameName(odom_frame_name_);
  tf_plugin_.setBaseFrameName(base_frame_name_);
  tf_plugin_.setRotationNoise(tf_trans_noise_);
  tf_plugin_.setRollNoise(tf_roll_noise_);
  tf_plugin_.setPitchNoise(tf_pitch_noise_);
  tf_plugin_.setYawNoise(tf_yaw_noise_);
  tf_plugin_.setTranslationNoise(tf_rot_noise_);
  if (use_tf_) {
    boost::shared_ptr<omnimapper::PosePlugin> tf_plugin_ptr(&tf_plugin_);
    omb_.addPosePlugin(tf_plugin_ptr);
  }

  // Add the No Motion Plugin (null motion model)
  if (use_no_motion_) {
    boost::shared_ptr<omnimapper::PosePlugin> no_motion_ptr(&no_motion_plugin_);
    omb_.addPosePlugin(no_motion_ptr);
  }

  // Set up a sensor_to_base functor, for plugins to use
  omnimapper::GetTransformFunctorPtr rgbd_to_base_ptr(
      new omnimapper::GetTransformFunctorTF(rgbd_frame_name_, base_frame_name_,
                                            ros_node_, tf_buffer_));
  // Optionally disable this, if we don't have TF available
  if (!use_rgbd_sensor_base_tf_functor_) {
    rgbd_to_base_ptr = omnimapper::GetTransformFunctorPtr(
        new omnimapper::GetTransformFunctorIdentity());
  }

  // Set up an ICP Plugin
  icp_plugin_.setUseGICP(true);
  icp_plugin_.setOverwriteTimestamps(false);
  icp_plugin_.setAddIdentityOnFailure(icp_add_identity_on_fail_);
  icp_plugin_.setShouldDownsample(true);
  icp_plugin_.setLeafSize(icp_leaf_size_);  // 0.02
  icp_plugin_.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp_plugin_.setScoreThreshold(icp_score_thresh_);
  icp_plugin_.setTransNoise(icp_trans_noise_);  // 10.1
  icp_plugin_.setRotNoise(icp_rot_noise_);      // 10.1
  icp_plugin_.setAddLoopClosures(icp_add_loop_closures_);
  icp_plugin_.setTransNoise(icp_trans_noise_);  // 10.1
  icp_plugin_.setRotNoise(icp_rot_noise_);      // 10.1
  icp_plugin_.setLoopClosureDistanceThreshold(
      icp_loop_closure_distance_threshold_);
  icp_plugin_.setSaveFullResClouds(icp_save_full_res_clouds_);
  icp_plugin_.setSensorToBaseFunctor(rgbd_to_base_ptr);

  // Set up edge ICP plugin
  edge_icp_plugin_.setUseGICP(false);
  edge_icp_plugin_.setOverwriteTimestamps(false);
  edge_icp_plugin_.setAddIdentityOnFailure(occ_edge_add_identity_on_fail_);
  edge_icp_plugin_.setShouldDownsample(false);
  edge_icp_plugin_.setMaxCorrespondenceDistance(
      occ_edge_max_correspondence_dist_);
  edge_icp_plugin_.setScoreThreshold(occ_edge_score_thresh_);
  edge_icp_plugin_.setTransNoise(occ_edge_trans_noise_);  // 10.1
  edge_icp_plugin_.setRotNoise(occ_edge_rot_noise_);      // 10.1
  edge_icp_plugin_.setAddLoopClosures(false);
  edge_icp_plugin_.setLoopClosureDistanceThreshold(0.15);
  edge_icp_plugin_.setSaveFullResClouds(false);
  edge_icp_plugin_.setSensorToBaseFunctor(rgbd_to_base_ptr);

  // Set up the feature extraction
  if (use_occ_edge_icp_) {
    boost::function<void(const CloudConstPtr&)> edge_icp_cloud_cb = boost::bind(
        &omnimapper::ICPPoseMeasurementPlugin<PointT>::cloudCallback,
        &edge_icp_plugin_, _1);
    organized_segmentation_.setOccludingEdgeCallback(edge_icp_cloud_cb);
  }

  // Optionally use planes in the visualizer
  if (ar_mode_) {
    boost::function<void(
        std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
        omnimapper::Time)>
        plane_vis_cb = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::planarRegionCallback,
            &vis_plugin_, _1, _2);
    organized_segmentation_.setPlanarRegionStampedCallback(plane_vis_cb);
  }

  // Optionally draw label cloud
  if (draw_label_cloud_) {
    boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)>
        label_vis_callback = boost::bind(
            &omnimapper::OmniMapperVisualizerRViz<PointT>::labelCloudCallback,
            &vis_plugin_, _1, _2);
    organized_segmentation_.setClusterLabelsCallback(label_vis_callback);
  }

  // Canonical Scan Matcher
  if (use_csm_) {
    // Subscribe to laser scan
    laserScan_sub_ =
        ros_node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
              this->laserScanCallback(msg);
            });

    // Make a Trigger Functor: TODO: param this
    omnimapper::TriggerFunctorPtr trigger_functor_ptr(
        new omnimapper::TriggerAlways());
    csm_plugin_.setTriggerFunctor(trigger_functor_ptr);

    boost::shared_ptr<
        omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::msg::LaserScan> >
        csm_ptr(&csm_plugin_);
    csm_vis_plugin_.setCSMPlugin(csm_ptr);

    // Install the visualizer
    boost::shared_ptr<omnimapper::OutputPlugin> csm_vis_ptr(&csm_vis_plugin_);
    omb_.addOutputPlugin(csm_vis_ptr);

    boost::thread csm_thread(&omnimapper::CanonicalScanMatcherPlugin<
                                 sensor_msgs::msg::LaserScan>::spin,
                             &csm_plugin_);
  }

  // Set the ICP Plugin on the visualizer
  boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_ptr(
      &icp_plugin_);
  vis_plugin_.setICPPlugin(icp_ptr);

  // Subscribe to Point Clouds
  pointcloud_sub_ =
      ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          cloud_topic_name_, 1,
          [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->cloudCallback(msg);
          });

  // Install the visualizer
  if (use_rviz_plugin_) {
    vis_plugin_.setDrawPoseArray(draw_pose_array_);
    vis_plugin_.setDrawPoseGraph(draw_pose_graph_);
    vis_plugin_.setDrawICPCloudsAlways(draw_icp_clouds_always_);
    boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr(&vis_plugin_);
    omb_.addOutputPlugin(vis_ptr);
  }

  // OmniMapper thread
  omb_.setDebug(false);
  boost::thread omb_thread(&omnimapper::OmniMapperBase::spin, &omb_);
  if (use_icp_)
    boost::thread icp_thread(
        &omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &icp_plugin_);
  if (use_occ_edge_icp_)
    boost::thread edge_icp_thread(
        &omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &edge_icp_plugin_);
  if (use_rviz_plugin_)
    boost::thread rviz_plugin_thread(
        &omnimapper::OmniMapperVisualizerRViz<PointT>::spin, &vis_plugin_);

  if (use_organized_segmentation_) organized_segmentation_.spin();

  if (debug_)
    RCLCPP_INFO(ros_node_->get_logger(),
                "OmniMapperROS: Constructor complete.");
}

template <typename PointT>
void OmniMapperROS<PointT>::loadROSParams() {
  // Load some params
  ros_node_->get_parameter_or("use_planes", use_planes_, true);
  ros_node_->get_parameter_or("use_bounded_planes", use_bounded_planes_, true);
  ros_node_->get_parameter_or("use_objects", use_objects_, true);
  ros_node_->get_parameter_or("use_csm", use_csm_, true);
  ros_node_->get_parameter_or("use_icp", use_icp_, true);
  ros_node_->get_parameter_or("use_occ_edge_icp", use_occ_edge_icp_, false);
  ros_node_->get_parameter_or("use_tf", use_tf_, true);
  ros_node_->get_parameter_or("use_error_plugin", use_error_plugin_, false);
  ros_node_->get_parameter_or("use_error_eval_plugin", use_error_eval_plugin_,
                              false);
  ros_node_->get_parameter_or("use_no_motion", use_no_motion_, false);
  ros_node_->get_parameter_or("odom_frame_name", odom_frame_name_,
                              std::string("odom"));
  ros_node_->get_parameter_or("base_frame_name", base_frame_name_,
                              std::string("camera_depth_optical_frame"));
  ros_node_->get_parameter_or("cloud_topic_name", cloud_topic_name_,
                              std::string("/throttled_points"));
  ros_node_->get_parameter_or("rgbd_frame_name", rgbd_frame_name_,
                              std::string("/camera_rgb_optical_frame"));
  ros_node_->get_parameter_or("icp_leaf_size", icp_leaf_size_, 0.05);
  ros_node_->get_parameter_or("icp_max_correspondence_distance",
                              icp_max_correspondence_distance_, 0.5);
  ros_node_->get_parameter_or("icp_score_thresh", icp_score_thresh_, 0.8);
  ros_node_->get_parameter_or("icp_trans_noise", icp_trans_noise_, 0.1);
  ros_node_->get_parameter_or("icp_rot_noise", icp_rot_noise_, 0.1);
  ros_node_->get_parameter_or("icp_add_identity_on_fail",
                              icp_add_identity_on_fail_, false);
  ros_node_->get_parameter_or("icp_add_loop_closures", icp_add_loop_closures_,
                              true);
  ros_node_->get_parameter_or("icp_loop_closure_distance_threshold",
                              icp_loop_closure_distance_threshold_, 1.0);
  ros_node_->get_parameter_or("icp_loop_closure_score_threshold",
                              icp_loop_closure_score_threshold_, 0.8);
  ros_node_->get_parameter_or("icp_loop_closure_pose_index_threshold",
                              icp_loop_closure_pose_index_threshold_, 20);
  ros_node_->get_parameter_or("icp_save_full_res_clouds",
                              icp_save_full_res_clouds_, false);
  ros_node_->get_parameter_or("occ_edge_trans_noise", occ_edge_trans_noise_,
                              0.1);
  ros_node_->get_parameter_or("occ_edge_rot_noise", occ_edge_rot_noise_, 0.1);
  ros_node_->get_parameter_or("occ_edge_score_thresh", occ_edge_score_thresh_,
                              0.1);
  ros_node_->get_parameter_or("occ_edge_max_correspondence_dist",
                              occ_edge_max_correspondence_dist_, 0.1);
  ros_node_->get_parameter_or("occ_edge_add_identity_on_fail",
                              occ_edge_add_identity_on_fail_, false);
  ros_node_->get_parameter_or("plane_range_threshold", plane_range_threshold_,
                              0.6);
  ros_node_->get_parameter_or("plane_angular_threshold",
                              plane_angular_threshold_, pcl::deg2rad(10.0));
  ros_node_->get_parameter_or("plane_range_noise", plane_range_noise_, 0.2);
  ros_node_->get_parameter_or("plane_angular_noise", plane_angular_noise_,
                              0.26);
  ros_node_->get_parameter_or("tf_trans_noise", tf_trans_noise_, 0.05);
  ros_node_->get_parameter_or("tf_rot_noise", tf_rot_noise_,
                              pcl::deg2rad(10.0));
  ros_node_->get_parameter_or("tf_roll_noise", tf_roll_noise_, tf_rot_noise_);
  ros_node_->get_parameter_or("tf_pitch_noise", tf_pitch_noise_, tf_rot_noise_);
  ros_node_->get_parameter_or("tf_yaw_noise", tf_yaw_noise_, tf_rot_noise_);
  ros_node_->get_parameter_or("use_init_pose", use_init_pose_, false);
  ros_node_->get_parameter_or("suppress_commit_window", suppress_commit_window_,
                              false);
  ros_node_->get_parameter_or("init_pose_from_tf", init_pose_from_tf_, false);
  ros_node_->get_parameter_or("init_x", init_x_, 0.0);
  ros_node_->get_parameter_or("init_y", init_y_, 0.0);
  ros_node_->get_parameter_or("init_z", init_z_, 0.0);
  ros_node_->get_parameter_or("init_qx", init_qx_, 0.0);
  ros_node_->get_parameter_or("init_qy", init_qy_, 0.0);
  ros_node_->get_parameter_or("init_qz", init_qz_, 0.0);
  ros_node_->get_parameter_or("init_qw", init_qw_, 1.0);
  ros_node_->get_parameter_or("use_rviz_plugin", use_rviz_plugin_, true);
  ros_node_->get_parameter_or("draw_pose_array", draw_pose_array_, true);
  ros_node_->get_parameter_or("draw_pose_graph", draw_pose_graph_, true);
  ros_node_->get_parameter_or("draw_label_cloud", draw_label_cloud_, true);
  ros_node_->get_parameter_or("draw_clusters", draw_clusters_, false);
  ros_node_->get_parameter_or("draw_icp_clouds_always", draw_icp_clouds_always_,
                              false);
  ros_node_->get_parameter_or("use_label_cloud", use_label_cloud_, true);
  ros_node_->get_parameter_or("add_pose_per_cloud", add_pose_per_cloud_, true);
  ros_node_->get_parameter_or("broadcast_map_to_odom", broadcast_map_to_odom_,
                              false);
  ros_node_->get_parameter_or("broadcast_current_pose", broadcast_current_pose_,
                              false);
  ros_node_->get_parameter_or("use_distortion_model", use_distortion_model_,
                              false);
  ros_node_->get_parameter_or("use_rgbd_sensor_base_tf_functor",
                              use_rgbd_sensor_base_tf_functor_, true);
  ros_node_->get_parameter_or("evaluation_mode", evaluation_mode_, false);
  ros_node_->get_parameter_or("evaluation_pcd_path", evaluation_pcd_path_,
                              std::string(""));
  ros_node_->get_parameter_or("evaluation_associated_txt_path",
                              evaluation_associated_txt_path_, std::string(""));
  ros_node_->get_parameter_or("evaluation_ground_truth_txt_path",
                              evaluation_ground_truth_txt_path_,
                              std::string(""));
  ros_node_->get_parameter_or("evaluation_output_trajectory_txt_path",
                              evaluation_output_trajectory_txt_path_,
                              std::string(""));
  ros_node_->get_parameter_or("evaluation_mode_write_trajectory",
                              evaluation_mode_write_trajectory_, true);
  ros_node_->get_parameter_or("evaluation_mode_paused", evaluation_mode_paused_,
                              false);
  ros_node_->get_parameter_or("evaluation_show_frames", evaluation_show_frames_,
                              true);
  ros_node_->get_parameter_or("object_database_location",
                              object_database_location_,
                              std::string("/home/siddharth/kinect/"));
  ros_node_->get_parameter_or("object_loop_closures", object_loop_closures_,
                              true);
  ros_node_->get_parameter_or("object_landmarks", object_landmarks_, true);
  ros_node_->get_parameter_or("save_object_models", save_object_models_, true);
  ros_node_->get_parameter_or("object_min_height", object_min_height_, 0.3);
  ros_node_->get_parameter_or("use_organized_segmentation",
                              use_organized_segmentation_, true);
  ros_node_->get_parameter_or("debug", debug_, false);
  ros_node_->get_parameter_or("ar_mode", ar_mode_, false);
}

template <typename PointT>
void OmniMapperROS<PointT>::cloudCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  static uint32_t seq = 0;  // ROS2 is missing a seq field, we emulate this.
  if (debug_)
    RCLCPP_INFO(ros_node_->get_logger(), "OmniMapperROS got a cloud.");
  double start_cb = pcl::getTime();
  double start_copy = pcl::getTime();
  CloudPtr cloud(new Cloud());
  pcl::fromROSMsg<PointT>(*msg, *cloud);
  cloud->header.seq = seq++;  // Emulate the seq field.
  double end_copy = pcl::getTime();
  omnimapper::Time cloud_stamp = omnimapper::stamp2ptime(cloud->header.stamp);
  if (debug_) {
    std::cout << "OmniMapperRos: Got cloud from: " << cloud_stamp << std::endl;
  }

  if (debug_) {
    std::cout << "cloudCallback: conversion took "
              << double(end_copy - start_copy) << std::endl;
  }

  if (use_icp_) {
    if (debug_) {
      std::cout << "Calling ICP Plugin with stamp: "
                << omnimapper::stamp2ptime(cloud->header.stamp) << std::endl;
    }

    icp_plugin_.cloudCallback(cloud);
  }

  if (use_organized_segmentation_) {
    double start_ofe = pcl::getTime();
    organized_segmentation_.cloudCallback(cloud);
    double end_ofe = pcl::getTime();
    if (debug_)
      std::cout << "cloudCallback: ofe_cb took " << double(end_ofe - start_ofe)
                << std::endl;
  }

  if (add_pose_per_cloud_) {
    double start_getpose = pcl::getTime();
    gtsam::Symbol sym;
    boost::posix_time::ptime header_time = omnimapper::stamp2ptime(
        cloud->header.stamp);  // msg->header.stamp.toBoost ();
    if (debug_) std::cout << "header time: " << header_time << std::endl;
    omb_.getPoseSymbolAtTime(header_time, sym);
    double end_getpose = pcl::getTime();
    if (debug_)
      std::cout << "cloudCallback: get_pose took "
                << double(end_getpose - start_getpose) << std::endl;
  }
  if (broadcast_map_to_odom_) {
    double start_pub = pcl::getTime();
    publishMapToOdom();
    double end_pub = pcl::getTime();
    if (debug_)
      std::cout << "cloudCallback: pub took " << double(end_pub - start_pub)
                << std::endl;
  }
  if (broadcast_current_pose_) {
    publishCurrentPose();
  }
  double end_cb = pcl::getTime();
  if (debug_)
    std::cout << "cloudCallback: cb took: " << double(end_cb - start_cb)
              << std::endl;
}

template <typename PointT>
void OmniMapperROS<PointT>::laserScanCallback(
    sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
  if (debug_)
    RCLCPP_INFO(ros_node_->get_logger(), "OmniMapperROS: got a scan.");

  gtsam::Symbol sym;

  boost::shared_ptr<sensor_msgs::msg::LaserScan> lscanPtr1(
      new sensor_msgs::msg::LaserScan(*msg));
  csm_plugin_.laserScanCallback(lscanPtr1);
  publishMapToOdom();
}

template <typename PointT>
void OmniMapperROS<PointT>::publishMapToOdom() {
  gtsam::Pose3 current_pose;
  boost::posix_time::ptime current_time;
  omb_.getLatestPose(current_pose, current_time);

  const rclcpp::Time current_time_ros = omnimapper::ptime2rostime(current_time);
  const tf2::Transform current_pose_ros = omnimapper::pose3totf(current_pose);
  const tf2::Stamped<tf2::Transform> map_to_base(
      current_pose_ros.inverse(), tf2_ros::fromMsg(current_time_ros), "base");
  const geometry_msgs::msg::TransformStamped map_to_base_msg =
      tf2::toMsg<tf2::Stamped<tf2::Transform>,
                 geometry_msgs::msg::TransformStamped>(map_to_base);

  geometry_msgs::msg::TransformStamped odom_to_map_msg;
  tf2::Transform odom_to_map;

  try {
    const geometry_msgs::msg::TransformStamped base_to_odom =
        tf_buffer_->lookupTransform(odom_frame_name_, base_frame_name_,
                                    tf2_ros::fromMsg(current_time_ros),
                                    tf2::durationFromSec(0.05));
    tf2::doTransform(map_to_base_msg, odom_to_map_msg, base_to_odom);
    tf2::fromMsg(odom_to_map_msg.transform, odom_to_map);
  } catch (tf2::TransformException e) {
    RCLCPP_ERROR(ros_node_->get_logger(),
                 "OmniMapperROS: Error: could not immediately get odom to base "
                 "transform\n");
    odom_to_map.setIdentity();
    return;
  }
  tf2::Stamped<tf2::Transform> map_to_odom(
      odom_to_map.inverse(), tf2_ros::fromMsg(ros_node_->now()), "map");
  geometry_msgs::msg::TransformStamped map_to_odom_msg =
      tf2::toMsg<tf2::Stamped<tf2::Transform>,
                 geometry_msgs::msg::TransformStamped>(map_to_odom);
  map_to_odom_msg.child_frame_id = odom_frame_name_;
  tf_broadcaster_.sendTransform(map_to_odom_msg);
}

template <typename PointT>
void OmniMapperROS<PointT>::publishCurrentPose() {
  gtsam::Pose3 current_pose;
  boost::posix_time::ptime current_time;
  omb_.getLatestPose(current_pose, current_time);
  rclcpp::Time current_time_ros = omnimapper::ptime2rostime(current_time);

  tf2::Stamped<tf2::Transform> current_pose_ros(
      omnimapper::pose3totf(current_pose), tf2_ros::fromMsg(ros_node_->now()),
      "map");
  geometry_msgs::msg::TransformStamped current_pose_msg =
      tf2::toMsg<tf2::Stamped<tf2::Transform>,
                 geometry_msgs::msg::TransformStamped>(current_pose_ros);
  current_pose_msg.child_frame_id = "current_pose";
  tf_broadcaster_.sendTransform(current_pose_msg);
}

template <typename PointT>
void OmniMapperROS<PointT>::resetEvaluation() {
  evaluation_pcd_files_.clear();

  // ICP Plugin
  icp_plugin_.setUseGICP(true);
  icp_plugin_.setOverwriteTimestamps(false);
  icp_plugin_.setAddIdentityOnFailure(icp_add_identity_on_fail_);
  icp_plugin_.setShouldDownsample(true);
  icp_plugin_.setLeafSize(icp_leaf_size_);  // 0.02
  icp_plugin_.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp_plugin_.setScoreThreshold(icp_score_thresh_);
  icp_plugin_.setTransNoise(icp_trans_noise_);  // 10.1
  icp_plugin_.setRotNoise(icp_rot_noise_);      // 10.1
  icp_plugin_.setAddLoopClosures(icp_add_loop_closures_);
  icp_plugin_.setTransNoise(icp_trans_noise_);  // 10.1
  icp_plugin_.setRotNoise(icp_rot_noise_);      // 10.1
  icp_plugin_.setLoopClosureDistanceThreshold(
      icp_loop_closure_distance_threshold_);
  icp_plugin_.setSaveFullResClouds(true);
}

template class OmniMapperROS<pcl::PointXYZRGBA>;
