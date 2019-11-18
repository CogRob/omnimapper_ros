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

#pragma once

#include <omnimapper/omnimapper_base.h>

#include "geometry_msgs/msg/point.hpp"
#include "omnimapper_ros/canonical_scan_matcher_plugin.h"
#include "omnimapper_ros_msgs/srv/visualize_full_cloud.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace omnimapper {
template <typename LScanT>
class CSMVisualizerRViz : public omnimapper::OutputPlugin {
 public:
  CSMVisualizerRViz(omnimapper::OmniMapperBase* mapper);
  void update(boost::shared_ptr<gtsam::Values>& vis_values,
              boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph);
  void setCSMPlugin(
      boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<LScanT> >&
          csm_plugin) {
    csm_plugin_ = csm_plugin;
  }
  bool drawCSMMap(omnimapper_ros_msgs::srv::VisualizeFullCloud::Request& req,
                  omnimapper_ros_msgs::srv::VisualizeFullCloud::Response& res);

 protected:
  std::shared_ptr<rclcpp::Node> ros_node_;

  OmniMapperBase* mapper_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

  rclcpp::Service<omnimapper_ros_msgs::srv::VisualizeFullCloud>::SharedPtr
      draw_csm_map_srv_;

  boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<LScanT> >
      csm_plugin_;

  boost::shared_ptr<gtsam::Values> vis_values_;

  boost::shared_ptr<gtsam::NonlinearFactorGraph> vis_graph_;

  boost::mutex vis_mutex_;

  bool draw_graph_;

  bool draw_map_;
};

}  // namespace omnimapper
