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

#include <omnimapper/plugins/icp_plugin.h>
#include <omnimapper/plugins/plane_plugin.h>
#include <omnimapper/plugins/bounded_plane_plugin.h>
#include <omnimapper/plugins/no_motion_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/time.h>
#include <omnimapper/trigger.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

#include "omnimapper_ros/canonical_scan_matcher_plugin.h"
#include "omnimapper_ros/csm_visualizer.h"
#include "omnimapper_ros/get_transform_functor_tf.h"
#include "omnimapper_ros/omnimapper_visualizer_rviz.h"
#include "omnimapper_ros/ros_tf_utils.h"
#include "omnimapper_ros/tf_pose_plugin.h"
#include "omnimapper/organized_segmentation/organized_segmentation_tbb.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

template <typename PointT>
class OmniMapperROS {
  using Cloud = pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;
  using LabelCloud = typename pcl::PointCloud<pcl::Label>;
  using LabelCloudPtr = typename LabelCloud::Ptr;
  using LabelCloudConstPtr = typename LabelCloud::ConstPtr;

 public:
  // Constructor
  explicit OmniMapperROS(std::shared_ptr<rclcpp::Node> ros_node);

  // Load (or reload) ROS Parameters
  void loadROSParams();

  // Point Cloud Callback
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  // Laser Scan Callback
  void laserScanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

  // Evaluation Timer Callback
  // void evalTimerCallback (const ros::TimerEvent& e);

  // Map To Odometry Correction Publication
  void publishMapToOdom();

  // Publish the current pose in the map frame
  void publishCurrentPose();

  void resetEvaluation();

 protected:
  // ROS Node Handle
  std::shared_ptr<rclcpp::Node> ros_node_;

  // ROS QoS
  rclcpp::QoS ros_qos_;

  // TF Listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // TF Broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // OmniMapper Instance
  omnimapper::OmniMapperBase omb_;

  // TF Pose Plugin
  omnimapper::TFPosePlugin tf_plugin_;

  // No Motion Pose Plugin
  omnimapper::NoMotionPosePlugin no_motion_plugin_;

  // ICP Plugin
  omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin_;

  // Edge ICP Plugin
  omnimapper::ICPPoseMeasurementPlugin<PointT> edge_icp_plugin_;

  // Plane Plugin
  omnimapper::PlaneMeasurementPlugin<PointT> plane_plugin_;

  // Bounded Plane Plugin
  omnimapper::BoundedPlanePlugin<PointT> bounded_plane_plugin_;
  
  // CSM Plugin
  omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::msg::LaserScan>
      csm_plugin_;

  // Visualization
  omnimapper::OmniMapperVisualizerRViz<PointT> vis_plugin_;

  // CSM Visualization
  omnimapper::CSMVisualizerRViz<sensor_msgs::msg::LaserScan> csm_vis_plugin_;

  // Organized Feature Extraction
  cogrob::OrganizedSegmentationTBB<PointT> organized_segmentation_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_sub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_sub_;

  // Mapper config
  bool use_planes_;
  bool use_bounded_planes_;
  bool use_objects_;
  bool use_csm_;
  bool use_icp_;
  bool use_occ_edge_icp_;
  bool use_tf_;
  bool use_no_motion_;

  // ROS Params
  std::string odom_frame_name_;
  std::string base_frame_name_;
  std::string rgbd_frame_name_;
  std::string cloud_topic_name_;
  std::string ros_qos_type_;

  bool init_pose_from_tf_;
  bool use_init_pose_;
  bool suppress_commit_window_;
  double init_x_, init_y_, init_z_;
  double init_qx_, init_qy_, init_qz_, init_qw_;

  // ICP Params
  double icp_leaf_size_;
  double icp_max_correspondence_distance_;
  double icp_score_thresh_;
  double icp_trans_noise_;
  double icp_rot_noise_;
  bool icp_add_identity_on_fail_;
  bool icp_add_loop_closures_;
  double icp_loop_closure_distance_threshold_;
  double icp_loop_closure_score_threshold_;
  int icp_loop_closure_pose_index_threshold_;
  bool icp_save_full_res_clouds_;

  // Occluding Edge ICP Params
  double occ_edge_trans_noise_;
  double occ_edge_rot_noise_;
  double occ_edge_score_thresh_;
  double occ_edge_max_correspondence_dist_;
  bool occ_edge_add_identity_on_fail_;

  // Plane Plugin Params
  double plane_range_threshold_;
  double plane_angular_threshold_;
  double plane_range_noise_;
  double plane_angular_noise_;

  // Object Plugin Params
  std::string object_database_location_;
  bool object_loop_closures_;
  bool object_landmarks_;
  bool save_object_models_;
  double object_min_height_;

  // Labeled Cloud Plugin Params
  bool use_label_cloud_;

  // TF Plugin Params
  double tf_trans_noise_;
  double tf_rot_noise_;
  double tf_roll_noise_, tf_pitch_noise_, tf_yaw_noise_;

  // Visualization params
  bool use_rviz_plugin_;
  bool draw_pose_array_;
  bool draw_pose_graph_;
  bool draw_label_cloud_;
  bool draw_clusters_;
  bool draw_icp_clouds_always_;

  // Error plugin params
  bool use_error_plugin_;
  bool use_error_eval_plugin_;

  // Other Flags
  bool add_pose_per_cloud_;
  bool broadcast_map_to_odom_;
  bool broadcast_current_pose_;
  bool use_distortion_model_;
  bool use_rgbd_sensor_base_tf_functor_;
  std::string distortion_model_path_;
  bool debug_;
  bool ar_mode_;

  // Evaluation Mode
  bool evaluation_mode_;
  std::string evaluation_pcd_path_;
  std::vector<std::string> evaluation_pcd_files_;
  int evaluation_file_idx_;
  // Note: we require the associated_txt path because the PCD format does not
  // include timestamps. We instead load the timestamps from this file.
  std::string evaluation_associated_txt_path_;
  std::string evaluation_ground_truth_txt_path_;
  std::string evaluation_output_trajectory_txt_path_;
  bool evaluation_mode_write_trajectory_;
  bool evaluation_mode_paused_;
  bool use_organized_segmentation_;
  bool evaluation_show_frames_;
};
