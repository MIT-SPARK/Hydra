#pragma once
#include <config_utilities/config_utilities.h>
#include <hydra_multi/operators/dynamic_scene_graph_operator.h>
#include <hydra_multi/operators/mesh_operator.h>
#include <ianvs/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <hydra_msgs/msg/dsg_update.hpp>
#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <kimera_pgmo_msgs/srv/mesh_delta_query.hpp>
#include <memory>
#include <mutex>
#include <thread>

#include "hydra_multi/input/input.h"

namespace hydra_multi {

class RosDsgInput : public Input {
 public:
  using MeshDeltaQuery = kimera_pgmo_msgs::srv::MeshDeltaQuery;
  using MeshDeltaMsg = kimera_pgmo_msgs::msg::MeshDelta;
  using DsgUpdate = hydra_msgs::msg::DsgUpdate;
  using MeshDeltaPtr = std::unique_ptr<kimera_pgmo::MeshDelta>;

  struct Config {
    size_t dsg_queue_size = 100;
    size_t mesh_update_queue_size = 100;
    size_t queue_size = 100;
    float rate = 1.0;
    bool reliable_mesh = false;
    bool subscribe_to_mesh = true;
  } const config;

  RosDsgInput(const Config& config,
              UnitInterfaceState::Ptr state,
              std::string name,
              size_t id);

  ~RosDsgInput() = default;

  void init() override;

  void stop() override;

 private:
  struct StampedSceneGraphDelta {
    SceneGraphDelta update;
    uint64_t timestamp;
  };

  struct StampedMeshUpdate {
    MeshDeltaPtr update;
    uint64_t timestamp;
  };

  void spin();

  void processQueues();

  void dsgCallback(const DsgUpdate::ConstSharedPtr& msg);

  void meshUpdateCallback(const MeshDeltaMsg::ConstSharedPtr& msg);

  std::vector<MeshDeltaPtr> requestDeltas(
      const std::vector<uint16_t>& sequence_numbers);

  // Subscribers
  rclcpp::Subscription<DsgUpdate>::SharedPtr dsg_sub_;
  rclcpp::Subscription<MeshDeltaMsg>::SharedPtr mesh_update_sub_;

  // Service clients
  rclcpp::Client<MeshDeltaQuery>::SharedPtr mesh_update_client_;

  // Queues
  std::queue<StampedSceneGraphDelta> dsg_queue_;
  std::queue<StampedMeshUpdate> mesh_queue_;
  uint16_t mesh_sequence_ = 0;

  // Threading
  std::unique_ptr<std::thread> spin_thread_;
  std::mutex mutex_;

  // Registration
  inline static const auto registration_ =
      config::RegistrationWithConfig<Input,
                                     RosDsgInput,
                                     RosDsgInput::Config,
                                     UnitInterfaceState::Ptr,
                                     std::string,
                                     size_t>("RosDsgInput");
};

void declare_config(RosDsgInput::Config& config);

}  // namespace hydra_multi
