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
  std::chrono::nanoseconds timestamp;
};

using DescriptorFactoryFunc =
    std::function<Descriptor::Ptr(const DynamicSceneGraph&,
                                  const DynamicSceneGraphNode&)>;

Descriptor::Ptr makeAgentDescriptor(const DynamicSceneGraph& graph,
                                    const DynamicSceneGraphNode& agent_node);

struct ObjectDescriptorFactory {
  ObjectDescriptorFactory(double radius, size_t num_classes)
      : radius(radius), num_classes(num_classes) {}

  Descriptor::Ptr operator()(const DynamicSceneGraph& graph,
                             const DynamicSceneGraphNode& agent_node) const;

  const double radius;
  const size_t num_classes;
};

template <typename T>
struct HistogramConfig {
  HistogramConfig(T min, T max, size_t bins)
      : min(min), max(max), bins(bins), step((max - min) / bins) {}

  const T min;
  const T max;
  const size_t bins;
  const T step;

  size_t getBin(T value) const {
    if (value <= min) {
      return 0;
    }

    if (value >= max) {
      return bins - 1;
    }

    return std::floor((value - min) / step);
  }
};

struct PlaceDescriptorFactory {
  PlaceDescriptorFactory(double radius, const HistogramConfig<double>& config)
      : radius(radius), config(config) {}

  Descriptor::Ptr operator()(const DynamicSceneGraph& graph,
                             const DynamicSceneGraphNode& agent_node) const;

  const double radius;
  const HistogramConfig<double> config;
};

}  // namespace lcd
}  // namespace kimera
