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
#include <omnimapper/omnimapper_base.h>
#include <pcl/segmentation/planar_region.h>

#include "geometry_msgs/msg/point.hpp"
#include "omnimapper_ros_msgs/srv/publish_model.hpp"
#include "omnimapper_ros_msgs/srv/visualize_full_cloud.hpp"
#include "omnimapper_ros_msgs/srv/write_trajectory_file.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace omnimapper {
/** \brief OmniMapperVisualizerRViz is an output plugin for OmniMapper based on
 * the RViz.
 *
 * \author Alex Trevor
 */
template <typename PointT>
class OmniMapperVisualizerRViz : public omnimapper::OutputPlugin {
  using Cloud = typename pcl::PointCloud<PointT>;
  using CloudPtr = typename Cloud::Ptr;
  using CloudConstPtr = typename Cloud::ConstPtr;
  using LabelCloud = pcl::PointCloud<pcl::Label>;
  using LabelCloudPtr = typename LabelCloud::Ptr;
  using LabelCloudConstPtr = typename LabelCloud::ConstPtr;

 public:
  OmniMapperVisualizerRViz(omnimapper::OmniMapperBase* mapper,
                           std::shared_ptr<rclcpp::Node> ros_node,
                           std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  void update(boost::shared_ptr<gtsam::Values>& vis_values,
              boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
  void spinOnce();
  void spin();
  void planarRegionCallback(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
          regions,
      omnimapper::Time t);
  void drawBBox(
      pcl::PointCloud<pcl::PointXYZRGB>& cloud,
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
          marker_pub,
      int obj_idx);
  void labelCloudCallback(const CloudConstPtr& cloud,
                          const LabelCloudConstPtr& labels);
  void clusterCloudCallback(std::vector<CloudPtr> clusters, omnimapper::Time t,
                            boost::optional<std::vector<pcl::PointIndices> >);
  void setICPPlugin(omnimapper::ICPPoseMeasurementPlugin<PointT>* icp_plugin) {
    icp_plugin_ = icp_plugin;
  }
  bool drawICPCloudsCallback(
      const omnimapper_ros_msgs::srv::VisualizeFullCloud::Request& req,
      omnimapper_ros_msgs::srv::VisualizeFullCloud::Response& res);
  bool publishModel(const omnimapper_ros_msgs::srv::PublishModel::Request& req,
                    omnimapper_ros_msgs::srv::PublishModel::Response& res);
  void setDrawPoseArray(bool draw_pose_array) {
    draw_pose_array_ = draw_pose_array;
  }
  void setDrawPoseGraph(bool draw_pose_graph) {
    draw_pose_graph_ = draw_pose_graph;
  }
  void setOutputGraphviz(bool output_graphviz) {
    output_graphviz_ = output_graphviz;
  }
  void setDrawICPCloudsAlways(bool draw_always) {
    draw_icp_clouds_always_ = draw_always;
    if (draw_always) draw_icp_clouds_ = true;
  }
  void setDrawICPCloudsFullRes(bool full_res) {
    draw_icp_clouds_full_res_ = full_res;
  }

  bool writeTrajectoryFile(
      const omnimapper_ros_msgs::srv::WriteTrajectoryFile::Request& req,
      omnimapper_ros_msgs::srv::WriteTrajectoryFile::Response& res);

 protected:
  // A ROS Node and TF tools.
  std::shared_ptr<rclcpp::Node> ros_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // A reference to a mapper instance
  OmniMapperBase* mapper_;

  // Latest Map Information
  boost::mutex state_mutex_;
  boost::shared_ptr<gtsam::Values> vis_values_;
  boost::shared_ptr<gtsam::NonlinearFactorGraph> vis_graph_;
  bool updated_;

  // Publisher for the trajectory
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  // Publisher for the map clouds
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

  // Publisher for Planar boundaries
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      planar_boundary_pub_;

  // Publisher for visualization marker arrays
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pose_covariances_pub_;

  // Publishers for segmentation results of planes
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      segmented_plane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      segmented_label_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      segmented_clusters_pub_;

  // Publisher for object observations
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      object_observation_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      object_modeled_pub_;

  // Services.
  rclcpp::Service<omnimapper_ros_msgs::srv::WriteTrajectoryFile>::SharedPtr
      write_trajectory_srv_;
  rclcpp::Service<omnimapper_ros_msgs::srv::VisualizeFullCloud>::SharedPtr
      draw_icp_clouds_srv_;
  rclcpp::Service<omnimapper_ros_msgs::srv::PublishModel>::SharedPtr
      publish_model_srv_;

  // ICP Plugin Ref
  omnimapper::ICPPoseMeasurementPlugin<PointT>* icp_plugin_;

  std::vector<pcl::PlanarRegion<PointT>,
              Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
      latest_planes_;

  bool draw_icp_clouds_;

  bool draw_icp_clouds_always_;

  double draw_icp_clouds_interval_;

  double draw_icp_clouds_prev_time_;

  bool draw_icp_clouds_full_res_;

  bool draw_icp_clouds_downsampled_;

  bool draw_planar_landmarks_;

  bool draw_pose_array_;

  bool draw_pose_graph_;

  bool draw_object_observation_cloud_;

  bool draw_object_observation_bboxes_;

  bool draw_pose_marginals_;

  bool output_graphviz_;

  bool passthrough_filter_map_cloud_;

  bool write_trajectory_text_file_;
};
}  // namespace omnimapper
