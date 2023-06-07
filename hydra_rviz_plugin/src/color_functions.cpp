#include "hydra_rviz_plugin/color_functions.h"

namespace hydra {

namespace {

inline void fillColor(Ogre::ColourValue& lhs, const std::array<float, 3>& rhs) {
  lhs.r = rhs[0];
  lhs.g = rhs[1];
  lhs.b = rhs[2];
}

}  // namespace

using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraphNode;
using spark_dsg::SemanticNodeAttributes;

ColormapFunctor::ColormapFunctor(const Colormap& colormap, bool use_label)
    : colormap_(colormap), use_label_(use_label) {}

void ColormapFunctor::call(const SceneGraphNode& node, Ogre::ColourValue& color) {
  if (!use_label_) {
    auto rgb = colormap_(NodeSymbol(node.id).categoryId());
    fillColor(color, rgb);
    return;
  }

  const auto& attrs = node.attributes<SemanticNodeAttributes>();
  auto rgb = colormap_(attrs.semantic_label);
  fillColor(color, rgb);
}

ParentColorFunctor::ParentColorFunctor(const Colormap& colormap,
                                       DynamicSceneGraph::Ptr graph,
                                       bool use_label)
    : functor_(colormap, use_label), graph_(graph) {}

void ParentColorFunctor::call(const SceneGraphNode& node, Ogre::ColourValue& color) {
  if (!graph_) {
    return;
  }

  auto parent = node.getParent();
  if (!parent) {
    color.r = 0.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    return;
  }

  const auto& parent_node = graph_->getNode(*parent)->get();
  functor_(parent_node, color);
}

SingleColorFunctor::SingleColorFunctor(const std::array<float, 3>& color)
    : color_(color) {}

void SingleColorFunctor::call(const SceneGraphNode& node, Ogre::ColourValue& color) {
  fillColor(color, color_);
}

}  // namespace hydra
