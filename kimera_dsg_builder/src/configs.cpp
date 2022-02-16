#include "kimera_dsg_builder/configs.h"

namespace kimera {
namespace incremental {

std::map<std::string, bool> EnableMapConverter::from(
    const std::map<LayerId, bool>& other) const {
  std::map<std::string, bool> to_return;
  for (const auto& kv_pair : other) {
    to_return[KimeraDsgLayers::LayerIdToString(kv_pair.first)] = kv_pair.second;
  }

  return to_return;
}

std::map<LayerId, bool> EnableMapConverter::from(
    const std::map<std::string, bool>& other) const {
  std::map<LayerId, bool> to_return;
  for (const auto& kv_pair : other) {
    to_return[KimeraDsgLayers::StringToLayerId(kv_pair.first)] = kv_pair.second;
  }

  return to_return;
}

}  // namespace incremental
}  // namespace kimera
