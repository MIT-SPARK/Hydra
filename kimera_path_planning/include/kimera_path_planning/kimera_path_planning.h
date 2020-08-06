// NOTE: Most code is derived from mav_voxblox_planning:
// github.com/ethz-asl/mav_voxblox_planning
// Copyright (c) 2016, ETHZ ASL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of voxblox nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <ros/package.h>
#include <ros/ros.h>
#include <memory>
#include <string>

#include <kimera_semantics/common.h>

#include <mav_msgs/conversions.h>
#include <mav_path_smoothing/loco_smoother.h>
#include <mav_planning_common/physical_constraints.h>
#include <mav_planning_msgs/PlannerService.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_visualization/helpers.h>

#include <voxblox_planning_common/path_shortening.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_skeleton/ros/skeleton_vis.h>
#include <voxblox_skeleton/skeleton.h>
#include <voxblox_skeleton/skeleton_generator.h>
#include <voxblox_skeleton/skeleton_planner.h>
#include <voxblox_skeleton/sparse_graph_planner.h>

#include <voxblox_skeleton_planner/skeleton_graph_planner.h>

#include <kimera_scene_graph/scene_graph_server.h>

namespace kimera {

class SceneGraphGlobalPlanner {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SceneGraphGlobalPlanner(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);
  virtual ~SceneGraphGlobalPlanner() {}

  void generateSceneGraph();
  void generateSparseGraph();
  void setupSceneGraphPlanners();

  bool plannerServiceCallback(
      mav_planning_msgs::PlannerServiceRequest& request,
      mav_planning_msgs::PlannerServiceResponse& response);

  bool publishPathCallback(std_srvs::EmptyRequest& request,
                           std_srvs::EmptyResponse& response);

  void convertCoordinatePathToPath(
      const voxblox::AlignedVector<voxblox::Point>& coordinate_path,
      mav_msgs::EigenTrajectoryPointVector* path) const;

  double getMapDistance(const Eigen::Vector3d& position) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher path_marker_pub_;
  ros::Publisher skeleton_pub_;
  ros::Publisher sparse_graph_pub_;
  ros::Publisher path_pub_;
  ros::Publisher waypoint_list_pub_;

  ros::ServiceServer planner_srv_;
  ros::ServiceServer path_pub_srv_;

  // Settings for physical constriants.
  mav_planning::PhysicalConstraints constraints_;

  std::string scene_graph_path_;
  SceneGraph::Ptr scene_graph_;
  SceneGraphBuilder scene_graph_builder_;

  std::string frame_id_;
  bool visualize_;
  double voxel_size_;  // Cache the size of the voxels used by the map.

  std::string sparse_graph_path_;
  vxb::EsdfServer esdf_server_;
  // Loads the skeleton and creates the skeleton volumetric layer.
  voxblox::SkeletonGenerator skeleton_generator_;

  // Planners of all sorts.
  voxblox::SkeletonAStar skeleton_planner_; // Does A* on voxels
  mav_planning::SkeletonGraphPlanner skeleton_graph_planner_;
  mav_planning::EsdfPathShortener path_shortener_;
  mav_planning::LocoSmoother loco_smoother_;

  voxblox::SparseGraphPlanner places_skeleton_planner_;
  voxblox::SparseGraphPlanner rooms_skeleton_planner_;
  voxblox::SparseGraphPlanner buildings_skeleton_planner_;

  vxb::SparseSkeletonGraph places_skeleton_layer_;
  vxb::SparseSkeletonGraph rooms_skeleton_layer_;
  vxb::SparseSkeletonGraph buildings_skeleton_layer_;

  // Waypoints
  mav_msgs::EigenTrajectoryPointVector last_waypoints_;
};

}  // namespace kimera
