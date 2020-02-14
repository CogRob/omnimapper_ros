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

#include "omnimapper_ros/csm_visualizer.h"

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

template <typename LScanT>
omnimapper::CSMVisualizerRViz<LScanT>::CSMVisualizerRViz(
    omnimapper::OmniMapperBase* mapper, std::shared_ptr<rclcpp::Node> ros_node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer)
    : ros_node_(ros_node),
      tf_buffer_(tf_buffer),
      mapper_(mapper),
      vis_values_(new gtsam::Values()),
      vis_graph_(new gtsam::NonlinearFactorGraph()),
      draw_graph_(true),
      draw_map_(true) {
  pose_array_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseArray>(
      "trajectory", 0);

  marker_array_pub_ =
      ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/visualization_marker_array", 0);

  map_cloud_pub_ = ros_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "csm_map_cloud", 0);
}

template <typename LScanT>
void omnimapper::CSMVisualizerRViz<LScanT>::update(
    boost::shared_ptr<gtsam::Values>& vis_values,
    boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph) {
  {
    boost::lock_guard<boost::mutex> lock(vis_mutex_);
    vis_values_ = vis_values;
    vis_graph_ = vis_graph;
  }

  gtsam::Values current_solution = *vis_values;
  gtsam::NonlinearFactorGraph current_graph = *vis_graph;

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = ros_node_->now();

  pcl::PointCloud<pcl::PointXYZ>::Ptr aggregate_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  aggregate_cloud->header.frame_id = "map";
  // aggregate_cloud->header.stamp = ros_node_->now();

  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered =
      current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (
      const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value,
      pose_filtered) {
    geometry_msgs::msg::Pose pose;

    gtsam::Symbol key_symbol(key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion();

    // Eigen::Affine3d eigen_mat (pose.matrix ());
    // tf2::Transform tf_pose;
    // tf2::transformEigenToTF (eigen_mat, tf_pose);
    // X Y Z W
    tf2::Quaternion orientation(quat[1], quat[2], quat[3], quat[0]);
    pose.orientation = tf2::toMsg(orientation);
    pose.position.x = sam_pose.x();
    pose.position.y = sam_pose.y();
    pose.position.z = sam_pose.z();
    pose_array.poses.push_back(pose);

    if (draw_map_) {
      // Draw the scans too
      sensor_msgs::msg::PointCloud2 cloud_msg = csm_plugin_->getPC2(key_symbol);
      if (cloud_msg.width > 0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Matrix4f map_tform = sam_pose.matrix().cast<float>();
        pcl::transformPointCloud(*cloud, *map_cloud, map_tform);
        (*aggregate_cloud) += (*map_cloud);
      }
    }
  }

  pose_array_pub_->publish(pose_array);

  if (draw_map_) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*aggregate_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros_node_->now();
    map_cloud_pub_->publish(cloud_msg);
    // draw_map_ = false;
  }

  // Draw the graph
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

  BOOST_FOREACH (const gtsam::NonlinearFactorGraph::sharedFactor& factor,
                 current_graph) {
    // check for poses
    const gtsam::KeyVector keys = factor->keys ();

    // skip if there aren't two pose keys
    if ((keys.size() == 2)) {
      if ((gtsam::symbolChr(keys[0]) == 'x') &&
          (gtsam::symbolChr(keys[1]) == 'x')) {
        gtsam::Pose3 p1 = current_solution.at<gtsam::Pose3>(keys[0]);
        gtsam::Pose3 p2 = current_solution.at<gtsam::Pose3>(keys[1]);

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
    }
  }

  marker_array.markers.push_back(mapper_graph);
  marker_array_pub_->publish(marker_array);
}

template class omnimapper::CSMVisualizerRViz<sensor_msgs::msg::LaserScan>;
