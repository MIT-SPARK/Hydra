#include "hydra/common/label_remapper.h"

#include <config_utilities/parsing/yaml.h>
#include <glog/logging.h>

#include <filesystem>
#include <string>

namespace hydra {

void declare_config(LabelRemapRow& config) {
  config::name("LabelRemapRow");
  config::field(config.sub_id, "sub_id");
  config::field(config.super_id, "super_id");
}

LabelRemapper::LabelRemapper() {}

LabelRemapper::LabelRemapper(const std::string& remapping_file) {
  // Read the file.
  auto remappings = config::fromYamlFile<std::vector<LabelRemapRow>>(remapping_file);

  // Convert to map.
  for (size_t i = 0; i < remappings.size(); i++) {
    const auto& id_pair = remappings[i];
    label_remapping_[id_pair.sub_id] = id_pair.super_id;
  }
}

std::optional<uint32_t> LabelRemapper::remapLabel(const uint32_t from) const {
  const auto it = label_remapping_.find(from);
  if (it != label_remapping_.end()) {
    return it->second;
  }
  return std::nullopt;
}

}  // namespace hydra
