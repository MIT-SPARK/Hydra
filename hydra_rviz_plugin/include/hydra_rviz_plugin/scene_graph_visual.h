#include <memory>

namespace Ogre {

class SceneManager;
class SceneNode;
class Vector3;
class Quaternion;

}  // namespace Ogre

namespace rviz {

class PointCloud;
class BillboardLine;

}  // namespace rviz

namespace spark_dsg {

class DynamicSceneGraph;
class SceneGraphLayer;

}

namespace hydra {

class SceneGraphVisual {
 public:
  SceneGraphVisual(Ogre::SceneManager* manager, Ogre::SceneNode* parent);

  virtual ~SceneGraphVisual();

  void setMessage(const spark_dsg::DynamicSceneGraph& msg);

  void setPose(const Ogre::Vector3& pos, const Ogre::Quaternion& rot);

  void makeNodes(const spark_dsg::SceneGraphLayer& layer);

  void makeEdges(const spark_dsg::SceneGraphLayer& layer);

 private:
  Ogre::SceneManager* manager_;
  Ogre::SceneNode* node_;

  std::unique_ptr<rviz::PointCloud> graph_nodes_;
  std::unique_ptr<rviz::BillboardLine> graph_edges_;
};

}  // namespace hydra
