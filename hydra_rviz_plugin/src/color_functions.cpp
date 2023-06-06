#include "hydra_rviz_plugin/color_functions.h"

namespace hydra {

using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphNode;
using spark_dsg::SemanticNodeAttributes;

ColormapFunctor::ColormapFunctor(const Colormap& colormap, bool use_label)
    : colormap_(colormap), use_label_(use_label) {}

void ColormapFunctor::call(const SceneGraphNode& node, std::array<float, 3>& color) {
  if (!use_label_) {
    color = colormap_(NodeSymbol(node.id).categoryId());
    return;
  }

  const auto& attrs = node.attributes<SemanticNodeAttributes>();
  color = colormap_(attrs.semantic_label);
}

ParentColorFunctor::ParentColorFunctor(const Colormap& colormap,
                                       DynamicSceneGraph::Ptr graph,
                                       bool use_label)
    : functor_(colormap, use_label), graph_(graph) {}

void ParentColorFunctor::call(const SceneGraphNode& node, std::array<float, 3>& color) {
  if (!graph_) {
    return;
  }

  auto parent = node.getParent();
  if (!parent) {
    color = {0.0f, 0.0f, 0.0f};
    return;
  }

  const auto& parent_node = graph_->getNode(*parent)->get();
  functor_(parent_node, color);
}

SingleColorFunctor::SingleColorFunctor(const std::array<float, 3>& color)
    : color_(color) {}

void SingleColorFunctor::call(const SceneGraphNode& node, std::array<float, 3>& color) {
  color = color_;
}

}  // namespace hydra
