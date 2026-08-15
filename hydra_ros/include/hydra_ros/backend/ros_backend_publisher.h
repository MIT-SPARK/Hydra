/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <hydra/backend/backend_module.h>
#include <ianvs/node_handle.h>
#include <pose_graph_tools_ros/conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

class RosBackendPublisher : public BackendModule::Sink {
 public:
  struct Config {
    //! @brief Configuration for dsg publisher
    DsgSender::Config dsg_sender;
    //! @brief Publish odom to map transform
    bool publish_backend_tf = false;
    //! @brief Frame to use when publishing map_T_robot. An empty frame disables
    //! publishing
    std::string tf_pub_robot_frame = "";
    //! @brief Optional override for the map frame ID (defaults to current Hydra global
    //! setting)
    std::string tf_pub_map_frame = "";
    //! @brief Optional override for the odom frame ID (defaults to current Hydra global
    //! setting)
    std::string tf_pub_odom_frame = "";
  } const config;

  explicit RosBackendPublisher(ianvs::NodeHandle nh);

  virtual ~RosBackendPublisher() = default;

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph) const override;

  std::string printInfo() const override;

 protected:
  virtual void publishMeshGraph(const DynamicSceneGraph& graph,
                                const kimera_pgmo::DeformationGraph& dgraph,
                                const uint64_t& stamp) const;

  virtual void publishPoseGraph(const DynamicSceneGraph& graph,
                                const kimera_pgmo::DeformationGraph& dgraph,
                                const uint64_t& stamp) const;

  virtual void publishDeformationGraphViz(const kimera_pgmo::DeformationGraph& dgraph,
                                          size_t timestamp_ns) const;

  virtual void publishTf(const DynamicSceneGraph& graph,
                         const kimera_pgmo::DeformationGraph& dgraph) const;

 protected:
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_mesh_edges_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_mesh_edges_pub_;
  pose_graph_tools::PoseGraphPublisher pose_graph_pub_;
  pose_graph_tools::PoseGraphPublisher mesh_graph_pub_;
  std::unique_ptr<DsgSender> dsg_sender_;
  mutable tf2_ros::TransformBroadcaster tf_br_;
};

}  // namespace hydra
