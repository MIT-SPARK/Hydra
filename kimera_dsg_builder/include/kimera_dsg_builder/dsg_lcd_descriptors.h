#pragma once
#include <kimera_dsg/dynamic_scene_graph.h>

namespace kimera {
namespace lcd {

struct Descriptor {
  using Ptr = std::unique_ptr<Descriptor>;
  Eigen::Matrix<uint32_t, Eigen::Dynamic, 1> words;
  Eigen::VectorXf values;
  bool normalized = false;
  std::set<NodeId> nodes;
  NodeId root_node;
  Eigen::Vector3d root_position;
  std::chrono::nanoseconds timestamp;
};

struct DescriptorFactory {
  using Ptr = std::unique_ptr<DescriptorFactory>;

  virtual ~DescriptorFactory() = default;

  virtual Descriptor::Ptr construct(const DynamicSceneGraph& dsg,
                                    const DynamicSceneGraphNode& agent_node) const = 0;
};

struct AgentDescriptorFactory : DescriptorFactory {
  AgentDescriptorFactory() = default;

  virtual ~AgentDescriptorFactory() = default;

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;
};

struct ObjectDescriptorFactory : DescriptorFactory {
  ObjectDescriptorFactory(double radius, size_t num_classes);

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

  const double radius;
  const size_t num_classes;
};

template <typename T>
struct HistogramConfig {
  HistogramConfig(T min, T max, size_t bins) : min(min), max(max), bins(bins) {}

  T min;
  T max;
  size_t bins;

  size_t getBin(T value) const {
    if (value <= min) {
      return 0;
    }

    if (value >= max) {
      return bins - 1;
    }

    const T step = (max - min) / bins;
    return std::floor((value - min) / step);
  }
};

struct PlaceDescriptorFactory : DescriptorFactory {
  PlaceDescriptorFactory(double radius, const HistogramConfig<double>& config);

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

  const double radius;
  const HistogramConfig<double> config;
};

}  // namespace lcd
}  // namespace kimera
