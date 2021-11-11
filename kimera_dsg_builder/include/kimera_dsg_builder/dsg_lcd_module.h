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

  void updateDescriptorCache(incremental::SharedDsgInfo& dsg,
                             const std::unordered_set<NodeId>& archived_places);

  DsgRegistrationSolution detect(incremental::SharedDsgInfo& dsg,
                                 NodeId latest_agent_id);

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

 private:
  bool addNewDescriptors(const DynamicSceneGraph& graph,
                         const DynamicSceneGraphNode& agent_node);

  DsgRegistrationSolution registerAndVerify(
      incremental::SharedDsgInfo& dsg,
      const std::map<size_t, LayerSearchResults>& matches) const;

  DsgLcdConfig config_;
  std::map<LayerId, DescriptorFactoryFunc> layer_factories_;

  size_t max_internal_index_;
  std::map<LayerId, size_t> layer_to_internal_index_;
  std::map<size_t, DescriptorMatchConfig> match_config_map_;
  std::map<size_t, RegistrationFunc> registration_funcs_;
  std::map<size_t, ValidationFunc> validation_funcs_;

  std::map<LayerId, DescriptorCache> cache_map_;
  std::map<NodeId, DescriptorCache> leaf_cache_;
};

}  // namespace lcd
}  // namespace kimera