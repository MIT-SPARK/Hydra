#pragma once

#include "hydra/frontend/mesh_update_info.h"

namespace hydra {

class MeshSegmenter;

// Maintains object connections as compressed mesh vertices change or archive.
class MeshConnectionUpdater {
 public:
  void updateObjects(const MeshSegmenter& segmenter,
                     const MeshUpdateInfo& info,
                     spark_dsg::SceneGraph& graph);

 private:
  struct ObjectConnections {
    std::list<size_t> archived;
    // Source support for association; graph activity also includes vertices
    // that compression has not archived yet.
    bool has_active_support = true;
  };
  spatial_hash::IndexHashMap<const MeshBlock*> active_blocks_;
  std::map<const MeshBlock*, MeshUpdateInfo::BlockMapping> mappings_;
  std::map<spark_dsg::NodeId, ObjectConnections> objects_;
};

}  // namespace hydra
