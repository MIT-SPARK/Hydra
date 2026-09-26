#pragma once
#include <hydra/common/shared_dsg_info.h>

#include <map>
#include <memory>

#include "hydra_multi/common/types.h"

namespace hydra_multi {

class MultiDsgInfo : public hydra::SharedDsgInfo {
 public:
  using Ptr = std::shared_ptr<MultiDsgInfo>;

  explicit MultiDsgInfo(const hydra::SharedDsgInfo::Config& config);

  MultiDsgInfo::Ptr clone() const;

  void update(const MultiDsgInfo& dsg);

  void clear();

  void addRobotGraph(size_t robot_id,
                     const DynamicSceneGraph& dsg,
                     Eigen::Isometry3d* transform = nullptr);

 private:
  void remapAndAddLayer(size_t robot_id,
                        LayerKey layer_key,
                        const DynamicSceneGraph& dsg,
                        Eigen::Isometry3d* transform = nullptr);

  void remapAndAddLayers(size_t robot_id,
                         const DynamicSceneGraph& dsg,
                         Eigen::Isometry3d* transform = nullptr);

 public:
  hydra::SharedDsgInfo::Config config;
  RobotIdIdMap robot_node_map;
  NodeIdRobotMap node_robot_map;
  RobotIndexMap robot_vertex_offset;
  RobotIndexMap robot_num_vertices;
  std::map<LayerKey, size_t> layer_partition_next_node_idx;
};

}  // namespace hydra_multi
