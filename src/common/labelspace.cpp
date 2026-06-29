/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra/common/labelspace.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <string>

namespace {

struct LabelRemapRow {
  uint32_t sub_id;
  uint32_t super_id;
};

}  // namespace

namespace YAML {

template <>
struct convert<LabelRemapRow> {
  static Node encode(const LabelRemapRow& row) {
    Node node;
    node["sub_id"] = row.sub_id;
    node["super_id"] = row.super_id;
    return node;
  }

  static bool decode(const Node& node, LabelRemapRow& row) {
    row.sub_id = node["sub_id"].as<uint32_t>();
    row.super_id = node["super_id"].as<uint32_t>();
    return true;
  }
};

}  // namespace YAML

namespace hydra {
namespace {

struct LabelNameConversion {
  using YamlList = std::vector<std::map<std::string, std::string>>;
  using SourceMap = std::map<uint32_t, std::string>;

  static YamlList toIntermediate(const SourceMap& other, std::string&) {
    YamlList to_return;
    for (const auto& kv_pair : other) {
      std::map<std::string, std::string> value_map{
          {"label", std::to_string(kv_pair.first)}, {"name", kv_pair.second}};
      to_return.push_back(value_map);
    }

    return to_return;
  }

  static void fromIntermediate(const YamlList& other,
                               SourceMap& value,
                               std::string& error) {
    value.clear();
    for (const auto& value_map : other) {
      if (!value_map.count("name")) {
        error = "invalid format! missing key 'name'";
        break;
      }

      if (!value_map.count("label")) {
        error = "invalid format! missing key 'label'";
        break;
      }

      value[std::stoi(value_map.at("label"))] = value_map.at("name");
    }
  }
};

static const auto reg =
    config::RegistrationWithConfig<Labelspace, Labelspace, Labelspace>("from_config");

}  // namespace

void declare_config(Labelspace& config) {
  using namespace config;
  name("Labelspace");
  field(config.total_labels, "total_semantic_labels");
  field(config.dynamic_labels, "dynamic_labels");
  field(config.invalid_labels, "invalid_labels");
  field(config.object_labels, "object_labels");
  field(config.surface_places_labels, "surface_places_labels");
  field<LabelNameConversion>(config.label_names, "label_names");
}

LabelRemapper::LabelRemapper() {}

LabelRemapper::LabelRemapper(const std::filesystem::path& filepath) {
  if (!filepath.empty() && !std::filesystem::exists(filepath)) {
    LOG(ERROR) << "Invalid filepath " << filepath << " for label remapping!";
    return;
  }

  const auto node = YAML::LoadFile(filepath);
  const auto remappings = node.as<std::vector<LabelRemapRow>>();
  for (const auto& [sub, super] : remappings) {
    label_remapping_[sub] = super;
  }
}

std::optional<uint32_t> LabelRemapper::remapLabel(const uint32_t from) const {
  const auto it = label_remapping_.find(from);
  return it != label_remapping_.end() ? std::optional<uint32_t>(it->second)
                                      : std::nullopt;
}

bool LabelRemapper::empty() const { return label_remapping_.empty(); }

LabelRemapper::operator bool() const { return empty(); }

}  // namespace hydra
