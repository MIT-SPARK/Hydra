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
#include <hydra/common/dsg_types.h>
#include <ianvs/node_handle.h>

#include <optional>

#include <hydra_msgs/msg/dsg_update.hpp>
#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

namespace hydra {

class DsgSender {
 public:
  struct Config {
    //! Frame ID to publish scene graph under
    std::string frame_id;
    //! Timer name to record serialization timing
    std::string timer_name = "publish_dsg";
    //! Include mesh in serialized scene graph message
    bool serialize_dsg_mesh = true;
    //! Publish mesh separately from scene graph
    bool publish_mesh = false;
    //! Minimum amount of time separating mesh messages
    double min_mesh_separation_s = 0.0;
    //! Minimum amount of time separating scene graph messages
    double min_dsg_separation_s = 0.0;

    //! Create a config with a specific timer name
    Config with_name(const std::string& name) const;
    //! Create a config with a specific frame ID
    Config with_frame(const std::string& frame) const;
  } const config;

  DsgSender(const Config& config, ianvs::NodeHandle nh);

  DsgSender(ianvs::NodeHandle nh,
            const std::string& frame_id,
            const std::string& timer_name = "publish_dsg",
            bool publish_mesh = false,
            double min_mesh_separation_s = 0.0,
            bool serialize_dsg_mesh = true);

  void sendGraph(const DynamicSceneGraph& graph, const rclcpp::Time& stamp) const;

 private:
  void publishMesh(const DynamicSceneGraph& graph, uint64_t timestamp_ns) const;

  void publishGraph(const DynamicSceneGraph& graph, uint64_t timestamp_ns) const;

  rclcpp::Publisher<hydra_msgs::msg::DsgUpdate>::SharedPtr pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr mesh_pub_;
  mutable std::optional<uint64_t> last_mesh_time_ns_;
  mutable std::optional<uint64_t> last_dsg_time_ns_;
};

class DsgReceiver {
 public:
  using LogCallback = std::function<void(const rclcpp::Time&, size_t)>;

  DsgReceiver(ianvs::NodeHandle nh, bool subscribe_to_mesh = false);

  DsgReceiver(ianvs::NodeHandle nh, const LogCallback& cb);

  inline DynamicSceneGraph::Ptr graph() const { return graph_; }

  inline bool updated() const { return has_update_; }

  inline void clearUpdated() { has_update_ = false; }

 private:
  void handleUpdate(const hydra_msgs::msg::DsgUpdate::ConstSharedPtr& msg);

  void handleMesh(const kimera_pgmo_msgs::msg::Mesh::ConstSharedPtr& msg);

  rclcpp::Subscription<hydra_msgs::msg::DsgUpdate>::SharedPtr sub_;
  rclcpp::Subscription<kimera_pgmo_msgs::msg::Mesh>::SharedPtr mesh_sub_;

  bool has_update_;
  DynamicSceneGraph::Ptr graph_;
  Mesh::Ptr mesh_;

  std::unique_ptr<LogCallback> log_callback_;
};

void declare_config(DsgSender::Config& config);

}  // namespace hydra
