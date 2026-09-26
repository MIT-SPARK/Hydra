#pragma once
#include <hydra_multi/backend/module.h>
#include <hydra_ros/utils/dsg_streaming_interface.h>
#include <ianvs/node_handle.h>
#include <pose_graph_tools_ros/conversions.h>

#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace hydra_multi {

class MultiRosBackendPublisher : public MultiBackendModule::Sink {
 public:
  struct Config {
    hydra::DsgSender::Config dsg_sender;
  } const config;

  explicit MultiRosBackendPublisher(ianvs::NodeHandle nh);

  virtual ~MultiRosBackendPublisher() = default;

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const kimera_pgmo::DeformationGraph& dgraph,
            const MultiBackendModuleStatus& status) const override;

 protected:
  void publishPoseGraph(const DynamicSceneGraph& graph,
                        const kimera_pgmo::DeformationGraph& dgraph) const;

  void publishDeformationGraphViz(const kimera_pgmo::DeformationGraph& dgraph,
                                  uint64_t timestamp_ns) const;

 protected:
  ianvs::NodeHandle nh_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  pose_graph_tools::PoseGraphPublisher pose_graph_pub_;
  std::unique_ptr<hydra::DsgSender> dsg_sender_;
};

}  // namespace hydra_multi
