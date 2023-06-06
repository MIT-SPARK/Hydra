#pragma once
#include <spark_dsg/scene_graph_node.h>

#include <functional>
#include <memory>

namespace spark_dsg {

class SceneGraphLayer;

}  // namespace spark_dsg

namespace hydra {

struct ColorFunctor {
  using Ptr = std::unique_ptr<ColorFunctor>;

  virtual void call(const spark_dsg::SceneGraphNode& node,
                    std::array<float, 3>& color) = 0;

  inline void operator()(const spark_dsg::SceneGraphNode& node,
                         std::array<float, 3>& color) {
    call(node, color);
  }
};

}  // namespace hydra
