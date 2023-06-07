#pragma once
#include <spark_dsg/dynamic_scene_graph.h>

#include "hydra_rviz_plugin/colormap.h"
#include "hydra_rviz_plugin/common.h"

namespace hydra {

struct ColormapFunctor : public ColorFunctor {
  ColormapFunctor(const Colormap& colormap, bool use_label);

  virtual void call(const spark_dsg::SceneGraphNode& node,
                    Ogre::ColourValue& color) override;

  Colormap colormap_;
  bool use_label_;
};

struct ParentColorFunctor : public ColorFunctor {
  ParentColorFunctor(const Colormap& colormap,
                     spark_dsg::DynamicSceneGraph::Ptr graph,
                     bool use_label);

  virtual void call(const spark_dsg::SceneGraphNode& node,
                    Ogre::ColourValue& color) override;

  ColormapFunctor functor_;
  spark_dsg::DynamicSceneGraph::Ptr graph_;
};

struct SingleColorFunctor : public ColorFunctor {
  explicit SingleColorFunctor(const std::array<float, 3>& color);

  virtual void call(const spark_dsg::SceneGraphNode& node,
                    Ogre::ColourValue& color) override;

  std::array<float, 3> color_;
};

}  // namespace hydra
