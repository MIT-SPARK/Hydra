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
#include <hydra/frontend/graph_builder.h>
#include <pose_graph_tools_ros/conversions.h>

#include <map>

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <kimera_pgmo_msgs/srv/mesh_delta_query.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

class RosFrontendPublisher : public GraphBuilder::Sink {
 public:
  using MeshDeltaSrv = kimera_pgmo_msgs::srv::MeshDeltaQuery;
  using MeshDeltaRequest = kimera_pgmo_msgs::srv::MeshDeltaQuery::Request::SharedPtr;
  using MeshDeltaResponse = kimera_pgmo_msgs::srv::MeshDeltaQuery::Response::SharedPtr;
  using MeshDeltaMsg = kimera_pgmo_msgs::msg::MeshDelta;

  struct Config {
    //! @brief Configuration for dsg publisher
    DsgSender::Config dsg_sender;
    size_t mesh_delta_queue_size = 100;  // Store mesh delta to resend. 0 for infinite
  } const config;

  explicit RosFrontendPublisher(ianvs::NodeHandle);

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const BackendInput& backend_input) const override;

  std::string printInfo() const override;

 protected:
  void processMeshDeltaQuery(const MeshDeltaRequest req, MeshDeltaResponse resp);

  std::unique_ptr<DsgSender> dsg_sender_;
  pose_graph_tools::PoseGraphPublisher mesh_graph_pub_;
  rclcpp::Publisher<MeshDeltaMsg>::SharedPtr mesh_update_pub_;
  rclcpp::Service<MeshDeltaSrv>::SharedPtr mesh_delta_server_;

  mutable std::map<uint16_t, MeshDeltaMsg::SharedPtr> stored_delta_;
};

}  // namespace hydra
