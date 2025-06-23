#include "hydra/common/label_remapper.h"

#include <config_utilities/config.h>
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

namespace {

std::map<uint32_t, uint32_t> loadRemapping(const std::filesystem::path& remap_path) {
  if (remap_path.empty()) {
    return {};
  }

  if (!std::filesystem::exists(remap_path)) {
    LOG(ERROR) << "Label remappping file '" << remap_path.string()
               << "' does not exist";
    return {};
  }

  // Read the file.
  auto remappings = config::fromYamlFile<std::vector<LabelRemapRow>>(remap_path);

  // Convert to map.
  std::map<uint32_t, uint32_t> to_return;
  for (const auto& remap : remappings) {
    to_return[remap.sub_id] = remap.super_id;
  }

  return to_return;
}

}  // namespace

LabelRemapper::LabelRemapper() {}

LabelRemapper::LabelRemapper(const std::string& remapping_file)
    : LabelRemapper(loadRemapping(remapping_file)) {}

LabelRemapper::LabelRemapper(const std::map<uint32_t, uint32_t>& remapping)
    : label_remapping_(remapping) {}

std::optional<uint32_t> LabelRemapper::remapLabel(const uint32_t from) const {
  const auto it = label_remapping_.find(from);
  if (it != label_remapping_.end()) {
    return it->second;
  }
  return std::nullopt;
}

}  // namespace hydra
