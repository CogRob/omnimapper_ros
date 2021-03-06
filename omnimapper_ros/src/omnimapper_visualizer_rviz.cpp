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

#include <omnimapper/transform_tools.h>
#include <pcl/common/transforms.h>
#include <omnimapper/plane.h>
#include <omnimapper/BoundedPlane3.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"

#include <fstream>

#include "geometry_msgs/msg/pose_array.hpp"
#include "omnimapper_ros/omnimapper_visualizer_rviz.h"
#include "omnimapper_ros/ros_time_utils.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

template <typename PointT>
omnimapper::OmniMapperVisualizerRViz<PointT>::OmniMapperVisualizerRViz(
    omnimapper::OmniMapperBase* mapper, std::shared_ptr<rclcpp::Node> ros_node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : ros_node_(ros_node),
      tf_buffer_(tf_buffer),
      mapper_(mapper),
      vis_values_(new gtsam::Values()),
      vis_graph_(new gtsam::NonlinearFactorGraph()),
      updated_(false),
      draw_icp_clouds_(false),
      draw_icp_clouds_always_(false),
      draw_icp_clouds_interval_(5.0),
      draw_icp_clouds_prev_time_(pcl::getTime()),
      draw_icp_clouds_full_res_(false),
      draw_icp_clouds_downsampled_(true),
      draw_planar_landmarks_(true),
      draw_pose_array_(true),
      draw_pose_graph_(true),
      draw_object_observation_cloud_(true),
      draw_object_observation_bboxes_(true),
      draw_pose_marginals_(false),
      output_graphviz_(false),
      passthrough_filter_map_cloud_(false),
      write_trajectory_text_file_(false) {
  pose_array_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "trajectory", 0);

  map_cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_cloud", 0);

  planar_boundary_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "planar_boundaries", 0);

  marker_array_pub_ =
      ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/visualization_marker_array", 0);

  segmented_plane_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "segmented_planes", 0);

  segmented_label_cloud_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "segmented_label_cloud", 0);

  segmented_clusters_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "segmented_clusters", 0);

  object_observation_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "object_observations", 0);

  draw_icp_clouds_srv_ =
      ros_node_->create_service<omnimapper_ros_msgs::srv::VisualizeFullCloud>(
          "draw_icp_clouds",
          [this](const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<
                     omnimapper_ros_msgs::srv::VisualizeFullCloud::Request>
                     request,
                 const std::shared_ptr<
                     omnimapper_ros_msgs::srv::VisualizeFullCloud::Response>
                     response) {
            this->drawICPCloudsCallback(*request, *response);
          });

  publish_model_srv_ = ros_node_->create_service<
      omnimapper_ros_msgs::srv::PublishModel>(
      "publish_model",
      [this](
          const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<omnimapper_ros_msgs::srv::PublishModel::Request>
              request,
          const std::shared_ptr<
              omnimapper_ros_msgs::srv::PublishModel::Response>
              response) { this->publishModel(*request, *response); });

  write_trajectory_srv_ =
      ros_node_->create_service<omnimapper_ros_msgs::srv::WriteTrajectoryFile>(
          "write_trajectory",
          [this](const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<
                     omnimapper_ros_msgs::srv::WriteTrajectoryFile::Request>
                     request,
                 const std::shared_ptr<
                     omnimapper_ros_msgs::srv::WriteTrajectoryFile::Response>
                     response) {
            this->writeTrajectoryFile(*request, *response);
          });

  pose_covariances_pub_ =
      ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/pose_covariances", 0);

  object_modeled_pub_ =
      ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "modelled_objects", 0);
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::drawBBox(
    pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub,
    int obj_idx) {
  // Get bbox points
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);

  geometry_msgs::msg::Point p1;
  p1.x = min_pt[0];
  p1.y = min_pt[1];
  p1.z = min_pt[2];
  geometry_msgs::msg::Point p2;
  p2.x = max_pt[0];
  p2.y = min_pt[1];
  p2.z = min_pt[2];
  geometry_msgs::msg::Point p3;
  p3.x = min_pt[0];
  p3.y = max_pt[1];
  p3.z = min_pt[2];
  geometry_msgs::msg::Point p4;
  p4.x = max_pt[0];
  p4.y = max_pt[1];
  p4.z = min_pt[2];

  geometry_msgs::msg::Point p5;
  p5.x = min_pt[0];
  p5.y = min_pt[1];
  p5.z = max_pt[2];
  geometry_msgs::msg::Point p6;
  p6.x = max_pt[0];
  p6.y = min_pt[1];
  p6.z = max_pt[2];
  geometry_msgs::msg::Point p7;
  p7.x = min_pt[0];
  p7.y = max_pt[1];
  p7.z = max_pt[2];
  geometry_msgs::msg::Point p8;
  p8.x = max_pt[0];
  p8.y = max_pt[1];
  p8.z = max_pt[2];

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker bbox_marker;
  bbox_marker.header.frame_id = "map";
  bbox_marker.header.stamp = ros_node_->now();
  bbox_marker.ns = "object_bboxes";
  bbox_marker.id = obj_idx;
  bbox_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  bbox_marker.action = visualization_msgs::msg::Marker::ADD;

  bbox_marker.points.push_back(p1);
  bbox_marker.points.push_back(p2);
  bbox_marker.points.push_back(p1);
  bbox_marker.points.push_back(p3);
  bbox_marker.points.push_back(p2);
  bbox_marker.points.push_back(p4);
  bbox_marker.points.push_back(p3);
  bbox_marker.points.push_back(p4);

  bbox_marker.points.push_back(p5);
  bbox_marker.points.push_back(p6);
  bbox_marker.points.push_back(p5);
  bbox_marker.points.push_back(p7);
  bbox_marker.points.push_back(p6);
  bbox_marker.points.push_back(p8);
  bbox_marker.points.push_back(p7);
  bbox_marker.points.push_back(p8);

  bbox_marker.points.push_back(p1);
  bbox_marker.points.push_back(p5);
  bbox_marker.points.push_back(p2);
  bbox_marker.points.push_back(p6);
  bbox_marker.points.push_back(p3);
  bbox_marker.points.push_back(p7);
  bbox_marker.points.push_back(p4);
  bbox_marker.points.push_back(p8);

  bbox_marker.scale.x = 0.01;
  bbox_marker.color.a = 0.2;
  bbox_marker.color.r = 1.0;
  bbox_marker.color.g = 1.0;
  bbox_marker.color.b = 1.0;
  marker_array.markers.push_back(bbox_marker);
  marker_pub->publish(marker_array);
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::update(
    boost::shared_ptr<gtsam::Values>& vis_values,
    boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph) {
  boost::lock_guard<boost::mutex> lock(state_mutex_);
  vis_values_ = vis_values;
  vis_graph_ = vis_graph;
  updated_ = true;
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::spin() {
  while (true) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    spinOnce();
  }
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::spinOnce() {
  boost::shared_ptr<gtsam::Values> current_solution(new gtsam::Values());
  boost::shared_ptr<gtsam::NonlinearFactorGraph> current_graph(
      new gtsam::NonlinearFactorGraph());
  bool got_data = false;

  if (state_mutex_.try_lock()) {
    if (updated_) {
      current_solution = vis_values_;
      current_graph = vis_graph_;
      updated_ = false;
      got_data = true;
    }
    state_mutex_.unlock();
  }

  // Not updated, or could not get lock
  if (!got_data) return;

  // gtsam::Values current_solution = *vis_values;
  // gtsam::NonlinearFactorGraph current_graph = *vis_graph;

  // Optionally output a graphviz
  if (output_graphviz_) {
    std::ofstream of;
    of.open("/tmp/omnimapper_graph.dot");
    current_graph->saveGraph(of);
    of.close();
    output_graphviz_ = false;
  }

  if (draw_icp_clouds_always_ &&
      ((pcl::getTime() - draw_icp_clouds_prev_time_) >
       draw_icp_clouds_interval_))
    draw_icp_clouds_ = true;

  // Draw the cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr aggregate_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  // CloudPtr aggregate_cloud (new Cloud ());
  // aggregate_cloud->header.frame_id = "map";
  // aggregate_cloud->header.stamp = ros_node->now ();

  // Draw object cloud
  pcl::PointCloud<pcl::PointXYZRGB>
      aggregate_object_observation_cloud;  // (new
                                           // pcl::PointCloud<pcl::PointXYZRGB>
                                           // ());
  // aggregate_object_observation_cloud.header.frame_id = "map";
  // aggregate_object_observation_cloud.header.stamp = ros_node->now ();

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = ros_node_->now();

  unsigned char red[6] = {255, 0, 0, 255, 255, 0};
  unsigned char grn[6] = {0, 255, 0, 255, 0, 255};
  unsigned char blu[6] = {0, 0, 255, 0, 255, 255};
  int obj_id = 0;

  gtsam::Values::ConstFiltered<gtsam::Point3> object_filtered =
      current_solution->filter<gtsam::Point3>();
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Point3>::KeyValuePair&
          key_value,
      object_filtered) {}

  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      current_solution->filter<gtsam::Pose3>();
  gtsam::Values::ConstFiltered<gtsam::Point3> point_filtered =
      current_solution->filter<gtsam::Point3>();

  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    geometry_msgs::msg::Pose pose;

    gtsam::Symbol key_symbol(key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation();
    int pose_idx = key_symbol.index();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion();
    Eigen::Matrix4f map_tform = sam_pose.matrix().cast<float>();

    // Eigen::Affine3d eigen_mat (pose.matrix ());
    // tf::Transform tf_pose;
    // tf::transformEigenToTF (eigen_mat, tf_pose);
    // X Y Z W
    tf2::Quaternion orientation(quat[1], quat[2], quat[3], quat[0]);
    pose.orientation = toMsg(orientation);
    pose.position.x = sam_pose.x();
    pose.position.y = sam_pose.y();
    pose.position.z = sam_pose.z();
    if (draw_pose_array_) pose_array.poses.push_back(pose);

    // Optionally Draw clouds too
    if (draw_icp_clouds_) {
      CloudConstPtr frame_cloud;
      if (draw_icp_clouds_full_res_)
        frame_cloud = icp_plugin_->getFullResCloudPtr(key_symbol);
      else
        frame_cloud = icp_plugin_->getCloudPtr(key_symbol);
      // CloudConstPtr frame_cloud = icp_plugin_->getCloudPtr (key_symbol);
      // CloudConstPtr frame_cloud = icp_plugin_->getFullResCloudPtr
      // (key_symbol);
      char frame_name[1024];
      CloudPtr map_cloud(new Cloud());
      // pose.print ("SAM Pose: ");
      std::cout << "Map Tform: " << map_tform << std::endl;
      pcl::transformPointCloud(*frame_cloud, *map_cloud, map_tform);
      sprintf(frame_name, "x_%zu", key_symbol.index());
      printf("name: x_%zu\n", key_symbol.index());

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_map_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      copyPointCloud(*map_cloud, *rgb_map_cloud);

      //(*aggregate_cloud) += (*map_cloud);
      (*aggregate_cloud) += (*rgb_map_cloud);
    }

    // Optionally draw object observations
    /*
    if (draw_object_observation_cloud_)
    {
      // Get the observations from this location
      std::vector<CloudPtr> obs_clouds = object_plugin_->getObservations
    (key_symbol);

      pcl::PointCloud<pcl::PointXYZRGB> cluster;

      for (int i = 0; i < obs_clouds.size (); i++)
      {
        // Get the cluster
        pcl::copyPointCloud ((*(obs_clouds[i])), cluster);

        // for (int j = 0; j < cluster.points.size (); j++)
        // {
        //   cluster.points[j].r = (cluster.points[j].r + red[i%6]) / 2;
        //   cluster.points[j].g = (cluster.points[j].g + grn[i%6]) / 2;
        //   cluster.points[j].b = (cluster.points[j].b + blu[i%6]) / 2;
        // }

        // Move it to the map frame
        pcl::transformPointCloud (cluster, cluster, map_tform);

        // Optionally draw a bbox too
        if (draw_object_observation_bboxes_)
        {
          drawBBox (cluster, marker_array_pub_, ++obj_id);
        }

        aggregate_object_observation_cloud += cluster;
      }

    }
    */
  }

  // Publish the poses
  if (draw_pose_array_) pose_array_pub_->publish(pose_array);

  // Draw the pose graph
  if (draw_pose_graph_) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker mapper_graph;
    mapper_graph.header.frame_id = "map";
    mapper_graph.header.stamp = rclcpp::Time();
    mapper_graph.ns = "error_lines";
    mapper_graph.id = 0;
    mapper_graph.type = visualization_msgs::msg::Marker::LINE_LIST;
    mapper_graph.action = visualization_msgs::msg::Marker::ADD;
    mapper_graph.color.a = 0.5;
    mapper_graph.color.r = 1.0;
    mapper_graph.color.g = 0.0;
    mapper_graph.color.b = 0.0;
    mapper_graph.scale.x = 0.01;

    visualization_msgs::msg::Marker object_graph;
    object_graph.header.frame_id = "map";
    object_graph.header.stamp = rclcpp::Time();
    object_graph.ns = "object_lines";
    object_graph.id = 0;
    object_graph.type = visualization_msgs::msg::Marker::LINE_LIST;
    object_graph.action = visualization_msgs::msg::Marker::ADD;
    object_graph.color.a = 0.5;
    object_graph.color.r = 0.0;
    object_graph.color.g = 1.0;
    object_graph.color.b = 0.0;
    object_graph.scale.x = 0.01;

    visualization_msgs::msg::Marker object_object_graph;
    object_object_graph.header.frame_id = "map";
    object_object_graph.header.stamp = rclcpp::Time();
    object_object_graph.ns = "object_object_lines";
    object_object_graph.id = 0;
    object_object_graph.type = visualization_msgs::msg::Marker::LINE_LIST;
    object_object_graph.action = visualization_msgs::msg::Marker::ADD;
    object_object_graph.color.a = 1.0;
    object_object_graph.color.r = 1.0;
    object_object_graph.color.g = 0.0;
    object_object_graph.color.b = 0.0;
    object_object_graph.scale.x = 0.01;

    BOOST_FOREACH (const gtsam::NonlinearFactorGraph::sharedFactor& factor,
                   (*current_graph)) {
      // check for poses
      const gtsam::KeyVector keys = factor->keys ();

      // skip if there aren't two pose keys
      if ((keys.size() == 2)) {
        if ((gtsam::symbolChr(keys[0]) == 'x') &&
            (gtsam::symbolChr(keys[1]) == 'x')) {
          gtsam::Pose3 p1 = current_solution->at<gtsam::Pose3>(keys[0]);
          gtsam::Pose3 p2 = current_solution->at<gtsam::Pose3>(keys[1]);

          geometry_msgs::msg::Point p1_msg;
          p1_msg.x = p1.x();
          p1_msg.y = p1.y();
          p1_msg.z = p1.z();

          geometry_msgs::msg::Point p2_msg;
          p2_msg.x = p2.x();
          p2_msg.y = p2.y();
          p2_msg.z = p2.z();

          mapper_graph.points.push_back(p1_msg);
          mapper_graph.points.push_back(p2_msg);
        }

        if ((gtsam::symbolChr(keys[0]) == 'x') &&
            (gtsam::symbolChr(keys[1]) == 'o')) {
          gtsam::Pose3 p1 = current_solution->at<gtsam::Pose3>(keys[0]);
          gtsam::Point3 p2 = current_solution->at<gtsam::Point3>(keys[1]);

          p1.print("Current Pose:\n");
          p2.print("Current Object:\n");
          geometry_msgs::msg::Point p1_msg;
          p1_msg.x = p1.x();
          p1_msg.y = p1.y();
          p1_msg.z = p1.z();

          geometry_msgs::msg::Point p2_msg;
          p2_msg.x = p2.x();
          p2_msg.y = p2.y();
          p2_msg.z = p2.z();

          object_graph.points.push_back(p1_msg);
          object_graph.points.push_back(p2_msg);
        }

        if ((gtsam::symbolChr(keys[0]) == 'o') &&
            (gtsam::symbolChr(keys[1]) == 'o')) {
          gtsam::Point3 p1 = current_solution->at<gtsam::Point3>(keys[0]);
          gtsam::Point3 p2 = current_solution->at<gtsam::Point3>(keys[1]);

          p1.print("Object1:\n");
          p2.print("Object2:\n");
          geometry_msgs::msg::Point p1_msg;
          p1_msg.x = p1.x();
          p1_msg.y = p1.y();
          p1_msg.z = p1.z();

          geometry_msgs::msg::Point p2_msg;
          p2_msg.x = p2.x();
          p2_msg.y = p2.y();
          p2_msg.z = p2.z();

          object_object_graph.points.push_back(p1_msg);
          object_object_graph.points.push_back(p2_msg);
        }
      }
    }

    marker_array.markers.push_back(mapper_graph);
    marker_array.markers.push_back(object_graph);
    // marker_array.markers.push_back (object_object_graph);
    marker_array_pub_->publish(marker_array);
  }

  // Optionally draw the pose marginals
  if (draw_pose_marginals_) {
    // gtsam::Marginals marginals (*vis_graph, *vis_values);
    gtsam::Marginals marginals(*current_graph, *current_solution);

    visualization_msgs::msg::MarkerArray pose_cov_markers;

    BOOST_FOREACH (
        const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair&
            key_value,
        pose_filtered) {
      try {
        geometry_msgs::msg::Pose pose;

        gtsam::Symbol key_symbol(key_value.key);
        gtsam::Pose3 sam_pose = key_value.value;
        gtsam::Rot3 rot = sam_pose.rotation();

        gtsam::Matrix pose_cov = marginals.marginalCovariance(key_symbol);
        gtsam::Matrix u, v;
        gtsam::Vector s;
        gtsam::svd(pose_cov, u, s, v);

        tf2::Matrix3x3 btm(u(0, 0), u(0, 1), u(0, 2), u(1, 0), u(1, 1), u(1, 2),
                           u(2, 0), u(2, 1), u(2, 2));
        tf2::Quaternion btq;
        btm.getRotation(btq);

        visualization_msgs::msg::Marker pose_cov_marker;
        pose_cov_marker.header.frame_id = "map";
        pose_cov_marker.header.stamp = ros_node_->now();
        pose_cov_marker.type = visualization_msgs::msg::Marker::SPHERE;
        pose_cov_marker.action = visualization_msgs::msg::Marker::ADD;
        pose_cov_marker.ns = "pose_covariances";
        pose_cov_marker.id = key_symbol.index();
        pose_cov_marker.color.r = 0.0f;
        pose_cov_marker.color.g = 0.0f;
        pose_cov_marker.color.b = 1.0f;
        pose_cov_marker.color.a = 0.05;
        pose_cov_marker.pose.position.x = sam_pose.x();
        pose_cov_marker.pose.position.y = sam_pose.y();
        pose_cov_marker.pose.position.z = sam_pose.z();
        pose_cov_marker.pose.orientation.x = btq.x();
        pose_cov_marker.pose.orientation.y = btq.y();
        pose_cov_marker.pose.orientation.z = btq.z();
        pose_cov_marker.pose.orientation.w = btq.w();
        pose_cov_marker.scale.x = sqrt(s[0]);
        pose_cov_marker.scale.y = sqrt(s[1]);
        pose_cov_marker.scale.z = sqrt(s[2]);
        pose_cov_markers.markers.push_back(pose_cov_marker);
      } catch (std::out_of_range) {
      }
    }

    BOOST_FOREACH (
        const gtsam::Values::ConstFiltered<gtsam::Point3>::KeyValuePair&
            key_value,
        point_filtered) {
      try {
        geometry_msgs::msg::Pose pose;

        gtsam::Symbol key_symbol(key_value.key);
        gtsam::Point3 sam_point = key_value.value;
        // gtsam::Rot3 rot = sam_pose.rotation ();

        gtsam::Matrix pose_cov = marginals.marginalCovariance(key_symbol);
        gtsam::Matrix u, v;
        gtsam::Vector s;
        gtsam::svd(pose_cov, u, s, v);

        tf2::Matrix3x3 btm(u(0, 0), u(0, 1), u(0, 2), u(1, 0), u(1, 1), u(1, 2),
                           u(2, 0), u(2, 1), u(2, 2));
        tf2::Quaternion btq;
        btm.getRotation(btq);

        visualization_msgs::msg::Marker pose_cov_marker;
        pose_cov_marker.header.frame_id = "map";
        pose_cov_marker.header.stamp = ros_node_->now();
        pose_cov_marker.type = visualization_msgs::msg::Marker::SPHERE;
        pose_cov_marker.action = visualization_msgs::msg::Marker::ADD;
        pose_cov_marker.ns = "pose_covariances";
        pose_cov_marker.id = key_symbol.index();
        pose_cov_marker.color.r = 0.0f;
        pose_cov_marker.color.g = 0.0f;
        pose_cov_marker.color.b = 1.0f;
        pose_cov_marker.pose.position.x = sam_point.x();
        pose_cov_marker.color.a = 0.05;
        pose_cov_marker.pose.position.y = sam_point.y();
        pose_cov_marker.pose.position.z = sam_point.z();
        pose_cov_marker.pose.orientation.x = 0.0f;
        pose_cov_marker.pose.orientation.y = 0.0f;
        pose_cov_marker.pose.orientation.z = 0.0f;
        pose_cov_marker.pose.orientation.w = 0.0f;
        pose_cov_marker.scale.x = sqrt(s[0]);
        pose_cov_marker.scale.y = sqrt(s[1]);
        pose_cov_marker.scale.z = sqrt(s[2]);
        pose_cov_markers.markers.push_back(pose_cov_marker);
      } catch (std::out_of_range) {
      }
    }

    pose_covariances_pub_->publish(pose_cov_markers);
  }

  // Optionally publish the ICP Clouds
  if (draw_icp_clouds_) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    if (passthrough_filter_map_cloud_) {
      // CloudPtr filtered_cloud (new Cloud ());

      pcl::PassThrough<pcl::PointXYZRGB> passthrough;
      passthrough.setInputCloud(aggregate_cloud);
      passthrough.setFilterFieldName("z");
      passthrough.setFilterLimits(-0.2, 2.0);
      passthrough.filter(*filtered_cloud);
      aggregate_cloud = filtered_cloud;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    if (draw_icp_clouds_downsampled_) {
      pcl::VoxelGrid<pcl::PointXYZRGB> grid;
      double leaf_size = 0.005;
      grid.setLeafSize(leaf_size, leaf_size, leaf_size);
      grid.setInputCloud(aggregate_cloud);
      grid.filter(*filtered_cloud);
      pcl::toROSMsg(*filtered_cloud, cloud_msg);
    } else {
      pcl::toROSMsg(*aggregate_cloud, cloud_msg);
    }

    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros_node_->now();
    map_cloud_pub_->publish(cloud_msg);
    // if (!draw_icp_clouds_always_)
    draw_icp_clouds_ = false;
    draw_icp_clouds_prev_time_ = pcl::getTime();
  }

  // Draw object observations
  if (draw_object_observation_cloud_) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(aggregate_object_observation_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros_node_->now();
    object_observation_pub_->publish(cloud_msg);
    draw_object_observation_cloud_ = false;
  }

  // Draw Planar Landmarks
  
  if (draw_planar_landmarks_)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    CloudPtr plane_boundary_cloud (new Cloud ());
    gtsam::Values::ConstFiltered<gtsam::Plane<PointT> > plane_filtered =
  current_solution->filter<gtsam::Plane<PointT> >(); int id = 0; BOOST_FOREACH
  (const typename gtsam::Values::ConstFiltered<gtsam::Plane<PointT>
  >::KeyValuePair& key_value, plane_filtered)
    {
      // Draw the boundary
      Cloud lm_cloud = key_value.value.hull ();
      (*plane_boundary_cloud) += lm_cloud;

      for (int i = 0; i < lm_cloud.points.size (); i++)
      {
        if (!pcl::isFinite (lm_cloud.points[i]))
          printf ("Error!  Point is not finite!\n");
      }

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (lm_cloud, centroid);

      printf ("RViz Plugin: Cloud had %d points\n", lm_cloud.points.size ());
      printf ("RViz Plugin Centroid: %lf %lf %lf\n", centroid[0], centroid[1],
  centroid[2]);

      geometry_msgs::msg::Point start;
      start.x = centroid[0];
      start.y = centroid[1];
      start.z = centroid[2];
      geometry_msgs::msg::Point end;
      end.x = centroid[0] + key_value.value.a ();
      end.y = centroid[1] + key_value.value.b ();
      end.z = centroid[2] + key_value.value.c ();

      // Draw the normal
      visualization_msgs::msg::Marker normal_marker;
      normal_marker.header.frame_id = "map";
      normal_marker.header.stamp = rclcpp::Time();
      normal_marker.ns = "planar_normals";
      normal_marker.id = ++id;//key_value.key.index ();
      normal_marker.type = visualization_msgs::msg::Marker::ARROW;
      normal_marker.action = visualization_msgs::msg::Marker::ADD;
      normal_marker.points.push_back (start);
      normal_marker.points.push_back (end);
      normal_marker.pose.position.x = 0.0;
      normal_marker.pose.position.y = 0.0;
      normal_marker.pose.position.z = 0.0;
      normal_marker.pose.orientation.x = 0.0;
      normal_marker.pose.orientation.y = 0.0;
      normal_marker.pose.orientation.z = 0.0;
      normal_marker.pose.orientation.w = 1.0;
      normal_marker.scale.x = 0.025;
      normal_marker.scale.y = 0.05;
      normal_marker.scale.z = 0.1;
      normal_marker.color.a = 0.5;
      normal_marker.color.r = 1.0;
      normal_marker.color.g = 0.0;
      normal_marker.color.b = 0.0;
      marker_array.markers.push_back (normal_marker);
    }


    //marker_array.header.stamp = ros_node_->now ();
    //marker_array.header.frame_id = "odom";
    marker_array_pub_->publish(marker_array);

    if (plane_boundary_cloud->points.size () > 0)
    {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg (*plane_boundary_cloud, cloud_msg);
      cloud_msg.header.frame_id = "map";
      cloud_msg.header.stamp = ros_node_->now ();
      planar_boundary_pub_->publish(cloud_msg);
    }

  }
  

  // Draw Bounded Planar Landmarks
  
  if (draw_planar_landmarks_)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    CloudPtr plane_boundary_cloud (new Cloud ());
    gtsam::Values::ConstFiltered<omnimapper::BoundedPlane3<PointT> >
  plane_filtered = current_solution->filter<omnimapper::BoundedPlane3<PointT>
  >(); int id = 0; BOOST_FOREACH (const typename
  gtsam::Values::ConstFiltered<omnimapper::BoundedPlane3<PointT>
  >::KeyValuePair& key_value, plane_filtered)
    {
      // Draw the boundary
      //Cloud lm_cloud = key_value.value.hull ();
      CloudConstPtr lm_cloud (key_value.value.boundary ());
      (*plane_boundary_cloud) += *lm_cloud;//(*(key_value.value.boundary ()));

      Eigen::Vector4d plane_coeffs = key_value.value.planeCoefficients ();

      for (int i = 0; i < lm_cloud->points.size (); i++)
      {
        if (!pcl::isFinite (lm_cloud->points[i]))
          printf ("Error!  Point is not finite!\n");
      }

      Eigen::Vector4f centroid;
      pcl::compute3DCentroid (*lm_cloud, centroid);

      printf ("RViz Plugin: Cloud had %d points\n", lm_cloud->points.size ());
      printf ("RViz Plugin Centroid: %lf %lf %lf\n", centroid[0], centroid[1],
  centroid[2]);

      geometry_msgs::msg::Point start;
      start.x = centroid[0];
      start.y = centroid[1];
      start.z = centroid[2];
      geometry_msgs::msg::Point end;
      end.x = centroid[0] + plane_coeffs[0];//key_value.value.a ();
      end.y = centroid[1] + plane_coeffs[1];//key_value.value.b ();
      end.z = centroid[2] + plane_coeffs[2];//key_value.value.c ();

      // Draw the normal
      visualization_msgs::msg::Marker normal_marker;
      normal_marker.header.frame_id = "map";
      normal_marker.header.stamp = rclcpp::Time();
      normal_marker.ns = "planar_normals";
      normal_marker.id = ++id;//key_value.key.index ();
      normal_marker.type = visualization_msgs::msg::Marker::ARROW;
      normal_marker.action = visualization_msgs::msg::Marker::ADD;
      normal_marker.points.push_back (start);
      normal_marker.points.push_back (end);
      normal_marker.pose.position.x = 0.0;
      normal_marker.pose.position.y = 0.0;
      normal_marker.pose.position.z = 0.0;
      normal_marker.pose.orientation.x = 0.0;
      normal_marker.pose.orientation.y = 0.0;
      normal_marker.pose.orientation.z = 0.0;
      normal_marker.pose.orientation.w = 1.0;
      normal_marker.scale.x = 0.025;
      normal_marker.scale.y = 0.05;
      normal_marker.scale.z = 0.1;
      normal_marker.color.a = 0.5;
      normal_marker.color.r = 1.0;
      normal_marker.color.g = 0.0;
      normal_marker.color.b = 0.0;
      marker_array.markers.push_back (normal_marker);
    }


    //marker_array.header.stamp = ros_node_->now ();
    //marker_array.header.frame_id = "odom";
    marker_array_pub_->publish(marker_array);

    if (plane_boundary_cloud->points.size () > 0)
    {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg (*plane_boundary_cloud, cloud_msg);
      cloud_msg.header.frame_id = "map";
      cloud_msg.header.stamp = ros_node_->now ();
      planar_boundary_pub_->publish(cloud_msg);
    }

  }
  
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::planarRegionCallback(
    std::vector<pcl::PlanarRegion<PointT>,
                Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
        regions,
    omnimapper::Time t) {
  latest_planes_ = regions;
  return;

  // Display the segmented planar regions
  pcl::PointCloud<PointT> aggregate_cloud;
  for (int i = 0; i < regions.size(); i++) {
    pcl::PointCloud<PointT> border_cloud;
    std::vector<PointT, Eigen::aligned_allocator<PointT> > border =
        regions[i].getContour();
    border_cloud.points = border;
    aggregate_cloud += border_cloud;
  }

  if (aggregate_cloud.points.size() > 0) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(aggregate_cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_rgb_optical_frame";
    cloud_msg.header.stamp = ptime2rostime(t);
    segmented_plane_pub_->publish(cloud_msg);
  }
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::labelCloudCallback(
    const CloudConstPtr& cloud, const LabelCloudConstPtr& labels) {
  // Create a colored label cloud
  pcl::PointCloud<pcl::PointXYZRGB> labeled_cloud;
  // labeled_cloud = (*cloud);
  pcl::copyPointCloud(*cloud, labeled_cloud);

  unsigned char red[6] = {255, 0, 0, 255, 255, 0};
  unsigned char grn[6] = {0, 255, 0, 255, 0, 255};
  unsigned char blu[6] = {0, 0, 255, 0, 255, 255};

  for (int i = 0; i < labeled_cloud.points.size(); i++) {
    labeled_cloud.points[i].r =
        (labeled_cloud.points[i].r + red[labels->points[i].label % 6]) / 2;
    labeled_cloud.points[i].g =
        (labeled_cloud.points[i].g + grn[labels->points[i].label % 6]) / 2;
    labeled_cloud.points[i].b =
        (labeled_cloud.points[i].b + blu[labels->points[i].label % 6]) / 2;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(labeled_cloud, cloud_msg);
  cloud_msg.header.frame_id = cloud->header.frame_id;
  cloud_msg.header.stamp = ptime2rostime(stamp2ptime(cloud->header.stamp));
  segmented_label_cloud_pub_->publish(cloud_msg);
}

template <typename PointT>
void omnimapper::OmniMapperVisualizerRViz<PointT>::clusterCloudCallback(
    std::vector<CloudPtr> clusters, omnimapper::Time t,
    boost::optional<std::vector<pcl::PointIndices> > indices) {
  printf("Omnimappervisualizerrviz: Got %zu clusters\n", clusters.size());

  if (clusters.size() == 0) return;

  pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;

  unsigned char red[6] = {255, 0, 0, 255, 255, 0};
  unsigned char grn[6] = {0, 255, 0, 255, 0, 255};
  unsigned char blu[6] = {0, 0, 255, 0, 255, 255};

  // color the clouds
  pcl::PointCloud<pcl::PointXYZRGB> color_cluster;
  for (int i = 0; i < clusters.size(); i++) {
    // if (clusters[i]->points.size () > 0)
    // aggregate_cloud += ((*(clusters[i])));

    if (clusters[i]->points.size() > 0) {
      printf("Cluster %d has %zu points\n", i, clusters[i]->points.size());
      color_cluster.resize(clusters[i]->points.size());
      pcl::copyPointCloud((*(clusters[i])), color_cluster);
      for (int j = 0; j < color_cluster.points.size(); j++) {
        color_cluster.points[j].r =
            (color_cluster.points[j].r + red[i % 6]) / 2;
        color_cluster.points[j].g =
            (color_cluster.points[j].g + grn[i % 6]) / 2;
        color_cluster.points[j].b =
            (color_cluster.points[j].b + blu[i % 6]) / 2;
      }

      try {
        aggregate_cloud += color_cluster;
      } catch (std::bad_alloc& ba) {
        std::cerr << "bad_alloc caught in omnimapper_rviz: " << ba.what()
                  << std::endl;
      }
    }
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(aggregate_cloud, cloud_msg);
  // moveFromPCL (aggregate_cloud, cloud_msg);
  cloud_msg.header.frame_id = "camera_rgb_optical_frame";
  cloud_msg.header.stamp = ptime2rostime(t);
  segmented_clusters_pub_->publish(cloud_msg);
}

/*
template<typename PointT> void
omnimapper::OmniMapperVisualizerRViz<PointT>::objectCallback (
std::map<gtsam::Symbol, Object<PointT> > object_map, gtsam::Point3 view_center,
gtsam::Point3 view_direction)
{

  typename std::map<gtsam::Symbol, Object<PointT> >::iterator it;
  typename std::map<gtsam::Symbol, CloudPtr>::iterator it_cluster;
  typename std::multimap<int, gtsam::Symbol> top_objects;
  typename std::multimap<int, gtsam::Symbol>::iterator obj_iterator;
  pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> truncated_map_cloud;

  for (it = object_map.begin (); it != object_map.end (); it++)
  {
    gtsam::Symbol sym = it->first;
    top_objects.insert(std::pair<int,
gtsam::Symbol>(it->second.clusters_.size(), sym));
  }


  int object_count = 0;
  for (obj_iterator = top_objects.begin (); obj_iterator != top_objects.end ();
    obj_iterator++)
  {
   // if(object_count == 20)break;
    gtsam::Symbol sym = obj_iterator->second;
    Object<PointT>& object = object_map.at(sym);

    Cloud optimal_cloud = object.optimalCloud();
    pcl::copyPointCloud(optimal_cloud, truncated_map_cloud);
    aggregate_cloud = aggregate_cloud + truncated_map_cloud;

    object_count++; //counter to process only top K objects
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(aggregate_cloud, cloud_msg);
  cloud_msg.header.frame_id = "map"; ///camera_rgb_optical_frame";
  cloud_msg.header.stamp = ros_node_->now ();
  object_modeled_pub_->publish(cloud_msg);


  view_direction.print("[rviz] view direction: ");
  view_center.print("[rviz] view center");

  visualization_msgs::msg::Marker view_marker;
  view_marker.header.frame_id = "map";
  view_marker.header.stamp = ros_node_->now ();
  view_marker.ns = "camera_view";
  view_marker.id = 0;
  view_marker.type = visualization_msgs::msg::Marker::ARROW;
  view_marker.action = visualization_msgs::msg::Marker::ADD;
  view_marker.scale.x = 1.0;
  view_marker.scale.y = 1.0;
  view_marker.scale.z = 1.0;
  view_marker.color.a = 0.5;
  view_marker.color.r = 1.0;
  view_marker.color.g = 1.0;
  view_marker.color.b = 1.0;





  gtsam::Point3 transformed_view_center (view_center.x (), view_center.y (),
    view_center.z ());

  // publish the camera frustum
  int depth_limit = 3; // frustum culling at 3m
  double vertical_angle = (49/2)*M_PI/180; // kinect vertical FOV=49 degrees
  double horizontal_angle = (57/2)*M_PI/180; // kinect horizontal FOV = 57
  gtsam::Point3 frame_center = transformed_view_center +
depth_limit*view_direction; frame_center.print("[rviz] frame center");

  geometry_msgs::msg::Point arrow_start;
  arrow_start.x = transformed_view_center.x();
  arrow_start.y = transformed_view_center.y();
  arrow_start.z = transformed_view_center.z();

  geometry_msgs::msg::Point arrow_end;
  arrow_end.x = frame_center.x();
  arrow_end.y = frame_center.y();
  arrow_end.z = frame_center.z();

  view_marker.points.push_back(arrow_start);
  view_marker.points.push_back(arrow_end);


  geometry_msgs::msg::Point p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;

  geometry_msgs::msg::Point frame_c;
  frame_c.x = 0;
  frame_c.y = 0;
  frame_c.z = depth_limit;

  geometry_msgs::msg::Point p2;
  p2.x = depth_limit*tan(horizontal_angle);
  p2.y = depth_limit*tan(vertical_angle);
  p2.z = depth_limit;

  geometry_msgs::msg::Point p3;
  p3.x = -p2.x ; p3.y = p2.y; p3.z = p2.z;

  geometry_msgs::msg::Point p4;
  p4.x = p2.x;
  p4.y = -p2.y;
  p4.z = p2.z;

  geometry_msgs::msg::Point p5;
  p5.x = -p2.x;
  p5.y = -p2.y;
  p5.z = p2.z;

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker frustum_marker;
  frustum_marker.header.frame_id = "/camera_rgb_optical_frame";
  frustum_marker.header.stamp = ros_node_->now ();
  frustum_marker.ns = "camera_frustum";
  frustum_marker.id = 0;
  frustum_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  frustum_marker.action = visualization_msgs::msg::Marker::ADD;

  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p2);
  frustum_marker.points.push_back (p4);
  frustum_marker.points.push_back (p5);
  frustum_marker.points.push_back (p3);
  frustum_marker.points.push_back (p2);
  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p3);
  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p4);
  frustum_marker.points.push_back (p1);
  frustum_marker.points.push_back (p5);




  frustum_marker.scale.x = 0.04;
 // frustum_marker.scale.y = 1.0;
 // frustum_marker.scale.z = 1.0;
  frustum_marker.color.a = 0.5;
  frustum_marker.color.r = 1.0;
  frustum_marker.color.g = 1.0;
  frustum_marker.color.b = 1.0;


  marker_array.markers.push_back (view_marker);
  marker_array.markers.push_back (frustum_marker);
  marker_array_pub_->publish(marker_array);



}
*/

// template <typename PointT> void
// clusterLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr&
// labels)

template <typename PointT>
bool omnimapper::OmniMapperVisualizerRViz<PointT>::drawICPCloudsCallback(
    const omnimapper_ros_msgs::srv::VisualizeFullCloud::Request& req,
    omnimapper_ros_msgs::srv::VisualizeFullCloud::Response& res) {
  draw_icp_clouds_ = true;
  return (true);
}

// template <typename PointT> bool
// omnimapper::OmniMapperVisualizerRViz<PointT>::drawObjectObservationCloud
// (omnimapper_ros::VisualizeFullCloud::Request &req,
// omnimapper_ros::VisualizeFullCloud::Response &res)
// {
//   draw_object_observation_cloud_ = true;
//   return (true);
// }

template <typename PointT>
bool omnimapper::OmniMapperVisualizerRViz<PointT>::publishModel(
    const omnimapper_ros_msgs::srv::PublishModel::Request& req,
    omnimapper_ros_msgs::srv::PublishModel::Response& res) {
  // Figure out where to spawn the model, by plane intersection

  // Get z-axis of latest pose
  Eigen::Vector3f camera_origin(0.0, 0.0, 0.0);
  Eigen::Vector3f camera_z(0.0, 0.0, 1.0);

  // compute intersection of camera viewpoint with all planes in the scene
  std::vector<Eigen::Vector3f> intersections;
  for (int i = 0; i < latest_planes_.size(); i++) {
    Eigen::Vector4f model = latest_planes_[i].getCoefficients();
    Eigen::Vector3f centroid = latest_planes_[i].getCentroid();
    Eigen::Vector3f norm3(model[0], model[1], model[2]);
    double u = norm3.dot((centroid - camera_origin)) /
               norm3.dot((camera_z - camera_origin));
    Eigen::Vector3f intersection =
        camera_origin + u * (camera_z - camera_origin);
    intersections.push_back(intersection);
  }

  if (intersections.size() == 0) {
    RCLCPP_WARN(ros_node_->get_logger(),
                "OmniMapper RViz: No interesctions found!");
    return (false);
  }

  // Find the closest intersection
  Eigen::Vector3f closest_intersection(0.0, 0.0, 0.0);
  double closest_dist = 999999.9;
  int best_idx = 0;
  for (int i = 0; i < intersections.size(); i++) {
    if (intersections[i].norm() < closest_dist) {
      best_idx = i;
      closest_dist = intersections[i].norm();
      closest_intersection = intersections[i];
    }
  }

  // Test
  Eigen::Vector4f target_planef = latest_planes_[best_idx].getCoefficients();
  Eigen::Vector4d target_plane(target_planef[0], target_planef[1],
                               target_planef[2], target_planef[3]);
  Eigen::Vector4d to_align(0.0, 0.0, 1.0, 0.0);
  Eigen::Affine3d align = planarAlignmentTransform(target_plane, to_align);
  gtsam::Pose3 pose = transformToPose3(align.cast<float>());
  gtsam::Quaternion quat = pose.rotation().toQuaternion();

  visualization_msgs::msg::MarkerArray marker_array;

  // Draw the intersection
  visualization_msgs::msg::Marker intersect_marker;
  intersect_marker.header.frame_id = "oculus_optical";
  intersect_marker.header.stamp = rclcpp::Time();
  intersect_marker.ns = "intersection";
  // intersect_marker.mesh_resource =
  // "package://omnimapper_ros/mesh/r2_small.dae";
  // intersect_marker.mesh_use_embedded_materials = true;
  intersect_marker.id = 1;
  intersect_marker.type = visualization_msgs::msg::Marker::SPHERE;
  intersect_marker.action = visualization_msgs::msg::Marker::ADD;
  // mesh_marker.points.push_back (start);
  // mesh_marker.points.push_back (end);
  intersect_marker.pose.position.x = closest_intersection.x();
  intersect_marker.pose.position.y = closest_intersection.y();
  intersect_marker.pose.position.z = closest_intersection.z();
  intersect_marker.pose.orientation.x = quat.x();  // 0.0;
  intersect_marker.pose.orientation.y = quat.y();  // 0.0;
  intersect_marker.pose.orientation.z = quat.z();  // 0.0;
  intersect_marker.pose.orientation.w = quat.w();  // 1.0;
  intersect_marker.scale.x = 0.1;
  intersect_marker.scale.y = 0.1;
  intersect_marker.scale.z = 0.1;
  intersect_marker.color.a = 1.0;
  intersect_marker.color.r = 1.0;
  intersect_marker.color.g = 0.0;
  intersect_marker.color.b = 0.0;
  marker_array.markers.push_back(intersect_marker);

  // Draw the mesh
  visualization_msgs::msg::Marker mesh_marker;
  mesh_marker.header.frame_id = "oculus_optical";
  mesh_marker.header.stamp = rclcpp::Time();
  mesh_marker.ns = "mesh";
  mesh_marker.mesh_resource = "package://omnimapper_ros/mesh/r2_small.dae";
  mesh_marker.mesh_use_embedded_materials = true;
  mesh_marker.id = 1;
  mesh_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  mesh_marker.action = visualization_msgs::msg::Marker::ADD;
  // mesh_marker.points.push_back (start);
  // mesh_marker.points.push_back (end);
  mesh_marker.pose.position.x = closest_intersection.x();
  mesh_marker.pose.position.y = closest_intersection.y();
  mesh_marker.pose.position.z = closest_intersection.z();
  mesh_marker.pose.orientation.x = quat.x();  // 1.0;
  mesh_marker.pose.orientation.y = quat.y();  // 0.0;
  mesh_marker.pose.orientation.z = quat.z();  // 0.0;
  mesh_marker.pose.orientation.w = quat.w();  // 1.0;
  mesh_marker.scale.x = 0.3;
  mesh_marker.scale.y = 0.3;
  mesh_marker.scale.z = 0.3;
  // mesh_marker.color.a = 1.0;
  // mesh_marker.color.r = 1.0;
  // mesh_marker.color.g = 0.0;
  // mesh_marker.color.b = 0.0;
  marker_array.markers.push_back(mesh_marker);

  marker_array_pub_->publish(marker_array);

  return (true);
}

template <typename PointT>
bool omnimapper::OmniMapperVisualizerRViz<PointT>::writeTrajectoryFile(
    const omnimapper_ros_msgs::srv::WriteTrajectoryFile::Request& req,
    omnimapper_ros_msgs::srv::WriteTrajectoryFile::Response& res) {
  gtsam::Values current_solution =
      mapper_->getSolution();  // mapper_->getSolution ();

  std::ofstream optimized_poses;
  optimized_poses.open(req.filename.c_str());

  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    geometry_msgs::msg::Pose pose;

    gtsam::Symbol key_symbol(key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    omnimapper::Time time;
    mapper_->getTimeAtPoseSymbol(key_symbol, time);
    std::string sym_str = boost::lexical_cast<std::string>(key_symbol.chr()) +
                          boost::lexical_cast<std::string>(key_symbol.index());
    optimized_poses << sym_str << std::endl;
    optimized_poses << boost::posix_time::to_iso_extended_string(time)
                    << std::endl;
    optimized_poses << sam_pose.matrix() << std::endl;
    // optimized_poses <<
    // optimized_poses << sam_pose.matrix () << std::endl;
    // std::string pcd_fname = "/home/siddharth/kinect/pcd_files/"
    // + boost::lexical_cast<std::string> (key_symbol.chr ())
    // + boost::lexical_cast<std::string> (key_symbol.index ()) + ".pcd";
    // pcl::io::savePCDFileBinary (pcd_fname, *frame_cloud);
  }
  optimized_poses.close();

  return (true);
}

// template <typename PointT> void
// omnimapper::OmniMapperVisualizerRViz<PointT>::planarRegionCallback
// (std::vector<pcl::PlanarRegion<PointT>,
// Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions,
// omnimapper::Time t)
// {

// }

// template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZ>;
template class omnimapper::OmniMapperVisualizerRViz<pcl::PointXYZRGBA>;
