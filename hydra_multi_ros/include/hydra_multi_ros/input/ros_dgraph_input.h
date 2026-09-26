#pragma once
#include <config_utilities/config_utilities.h>
#include <hydra_multi/input/input.h>
#include <pose_graph_tools_ros/conversions.h>

#include <queue>
namespace hydra_multi {

class RosDGraphInput : public Input {
 public:
  struct Config {
    size_t mesh_graph_queue_size = 100;
    size_t pose_graph_queue_size = 100;
    float rate = 1.0;
  } const config;

  RosDGraphInput(const Config& config,
                 UnitInterfaceState::Ptr state,
                 std::string name,
                 size_t id);

  ~RosDGraphInput() = default;

  void init() override;

  void stop() override;

 private:
  struct StampedPoseGraph {
    pose_graph_tools::PoseGraph update;
    uint64_t timestamp;
  };

  void meshGraphCallback(const pose_graph_tools::PoseGraph& mesh_graph_msg);

  void poseGraphCallback(const pose_graph_tools::PoseGraph& pose_graph_msg);

  void spin();

  void processQueues();

  // Queues
  std::queue<StampedPoseGraph> pose_graph_queue_;
  std::queue<StampedPoseGraph> mesh_graph_queue_;

  // Threading
  std::unique_ptr<std::thread> spin_thread_;
  std::mutex mutex_;

 private:
  pose_graph_tools::PoseGraphSubscription mesh_graph_sub_;
  pose_graph_tools::PoseGraphSubscription pose_graph_sub_;

  inline static const auto registration =
      config::RegistrationWithConfig<Input,
                                     RosDGraphInput,
                                     RosDGraphInput::Config,
                                     UnitInterfaceState::Ptr,
                                     std::string,
                                     size_t>("RosDGraphInput");
};

void declare_config(RosDGraphInput::Config& config);
}  // namespace hydra_multi
