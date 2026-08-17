#pragma once
#include <config_utilities/virtual_config.h>

#include <map>

#include "hydra/backend/update_functions.h"
#include "hydra/common/node_matchers.h"
#include "hydra/openset/clustering/agglomerative_clustering.h"
#include "hydra/utils/id_tracker.h"

namespace hydra {

struct ComponentInfo {
  using Ptr = std::unique_ptr<ComponentInfo>;
  using Config = AgglomerativeClustering::ClusteringConfig;

  ComponentInfo(const Config& config,
                const EmbeddingGroup& tasks,
                const EmbeddingDistance& metric,
                const spark_dsg::SceneGraphLayer& segments,
                const std::vector<spark_dsg::NodeId>& nodes,
                double I_xy_full);

  AgglomerativeClustering::Workspace ws;
  std::vector<spark_dsg::NodeId> segments;
  std::vector<spark_dsg::NodeId> objects;
};

class IBObjectsUpdateFunctor : public UpdateFunctor {
 public:
  struct Config : VerbosityConfig {
    char id_prefix = 'O';
    std::string source_layer = spark_dsg::DsgLayers::SEGMENTS;
    std::string target_layer = spark_dsg::DsgLayers::OBJECTS;
    std::string parent_layer = spark_dsg::DsgLayers::PLACES;
    double min_segment_score = 0.2;
    double min_object_score = 0.2;
    config::VirtualConfig<EmbeddingGroup> tasks;
    AgglomerativeClustering::ClusteringConfig clustering;
    config::VirtualConfig<EmbeddingDistance> metric{CosineDistance::Config()};
    config::VirtualConfig<NodeMatcher> edge_checker{BBoxIntersectionMatcher::Config()};
  } const config;

  explicit IBObjectsUpdateFunctor(const Config& config);

  void call(const spark_dsg::SceneGraph& unmerged,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

  std::set<size_t> addSegmentEdges(spark_dsg::SceneGraph& graph) const;

  void clearActiveComponents(spark_dsg::SceneGraph& graph,
                             const std::set<size_t>& active) const;

  void detectObjects(spark_dsg::SceneGraph& segments) const;

  void updateActiveParents(spark_dsg::SceneGraph& graph) const;

 protected:
  EmbeddingGroup::Ptr tasks_;
  std::unique_ptr<EmbeddingDistance> metric_;
  std::unique_ptr<NodeMatcher> edge_checker_;

  IdTracker components_ids_;
  mutable NodeSymbol next_node_id_;
  mutable std::set<spark_dsg::NodeId> ignored_;
  mutable std::set<spark_dsg::NodeId> active_;
  mutable std::map<size_t, ComponentInfo::Ptr> components_;
  mutable std::map<spark_dsg::NodeId, size_t> node_to_component_;
};

void declare_config(IBObjectsUpdateFunctor::Config& config);

}  // namespace hydra
