#pragma once
#include <functional>
#include <memory>

#include "hydra_rviz_plugin/common.h"

namespace Ogre {

class SceneManager;
class SceneNode;

}  // namespace Ogre

namespace rviz {

class BillboardLine;

}  // namespace rviz

namespace spark_dsg {

class DynamicSceneGraph;

}  // namespace spark_dsg

namespace hydra {

struct LayerConfig;
struct LayerContainer;
struct EdgeConfig;

struct Edge {
  Ogre::Vector3 start;
  Ogre::Vector3 end;
  Ogre::ColourValue color;
  std::optional<Ogre::ColourValue> end_color;
};

class InterlayerEdgeVisual {
 public:
  InterlayerEdgeVisual(Ogre::SceneManager* const manager,
                       Ogre::SceneNode* const parent);

  virtual ~InterlayerEdgeVisual();

  void setMessage(const spark_dsg::DynamicSceneGraph& graph,
                  const std::map<LayerId, LayerContainer>& layers,
                  const std::map<LayerPair, EdgeConfig>& configs);

  void setPose(const Pose& pose);

 private:
  void fillEdgeContainers(const spark_dsg::DynamicSceneGraph& graph,
                          const std::map<LayerId, LayerContainer>& layers,
                          const std::map<LayerPair, EdgeConfig>& configs,
                          std::map<LayerPair, std::vector<Edge>>& edges);

  void makeEdges(const std::vector<Edge>& edges,
                 const EdgeConfig& config,
                 rviz::BillboardLine& edge_visual);

  void resetEdges(const std::map<LayerPair, std::vector<Edge>>& edges);

  Ogre::SceneManager* const manager_;
  Ogre::SceneNode* node_;

  std::map<LayerPair, std::unique_ptr<rviz::BillboardLine>> edges_;
};

}  // namespace hydra
