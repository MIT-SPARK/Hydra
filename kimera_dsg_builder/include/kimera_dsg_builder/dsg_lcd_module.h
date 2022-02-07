#pragma once
#include "kimera_dsg_builder/dsg_lcd_descriptors.h"
#include "kimera_dsg_builder/dsg_lcd_matching.h"
#include "kimera_dsg_builder/dsg_lcd_registration.h"

#include "kimera_dsg_builder/incremental_types.h"

namespace kimera {
namespace lcd {

struct DsgLcdConfig {
  std::vector<DescriptorMatchConfig> search_configs;
  DescriptorMatchConfig agent_search_config;
};

using ValidationFunc =
    std::function<bool(const DynamicSceneGraph&, const LayerSearchResults&)>;

class DsgLcdModule {
 public:
  DsgLcdModule(const DsgLcdConfig& config,
               const std::map<LayerId, DescriptorFactoryFunc>& layer_factories,
               const std::map<LayerId, RegistrationFunc>& layer_registration_funcs,
               const std::map<LayerId, ValidationFunc>& layer_validation_funcs);

  void updateDescriptorCache(const DynamicSceneGraph& dsg,
                             const std::unordered_set<NodeId>& archived_places,
                             uint64_t timestamp = 0);

  std::vector<DsgRegistrationSolution> detect(const DynamicSceneGraph& dsg,
                                              NodeId latest_agent_id,
                                              uint64_t timestamp = 0);

  inline size_t numDescriptors() const {
    size_t num_descriptors = 0;
    for (const auto& id_cache_pair : cache_map_) {
      num_descriptors += id_cache_pair.second.size();
    }
    return num_descriptors + numAgentDescriptors();
  }

  inline size_t numGraphDescriptors(LayerId layer) const {
    if (!cache_map_.count(layer)) {
      return 0;
    }

    return cache_map_.at(layer).size();
  }

  inline size_t numAgentDescriptors() const {
    size_t count = 0;
    for (const auto& id_cache_pair : leaf_cache_) {
      count += id_cache_pair.second.size();
    }
    return count;
  }

  inline const std::map<size_t, LayerSearchResults>& getLatestMatches() const {
    return matches_;
  }

  inline const std::map<LayerId, size_t>& getLayerRemapping() const {
    return layer_to_internal_index_;
  }

  inline const DescriptorCache& getDescriptorCache(LayerId layer) {
    return cache_map_.at(layer);
  }

 private:
  bool addNewDescriptors(const DynamicSceneGraph& graph,
                         const DynamicSceneGraphNode& agent_node);

  std::vector<DsgRegistrationSolution> registerAndVerify(
      const DynamicSceneGraph& dsg,
      const std::map<size_t, LayerSearchResults>& matches,
      NodeId agent_node,
      uint64_t timestamp = 0) const;

  DsgLcdConfig config_;
  std::map<LayerId, DescriptorFactoryFunc> layer_factories_;

  size_t max_internal_index_;
  std::map<LayerId, size_t> layer_to_internal_index_;
  std::map<size_t, DescriptorMatchConfig> match_config_map_;
  std::map<size_t, RegistrationFunc> registration_funcs_;
  std::map<size_t, ValidationFunc> validation_funcs_;

  std::map<LayerId, DescriptorCache> cache_map_;
  std::map<NodeId, DescriptorCache> leaf_cache_;
  std::map<NodeId, std::set<NodeId>> root_leaf_map_;

  std::map<size_t, LayerSearchResults> matches_;
};

}  // namespace lcd
}  // namespace kimera
