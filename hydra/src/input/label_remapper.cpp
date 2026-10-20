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
#include "hydra/input/label_remapper.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <yaml-cpp/yaml.h>

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

static const auto reg =
    config::RegistrationWithConfig<InputAdapter, LabelRemapper, LabelRemapper::Config>(
        "LabelRemapper");

}  // namespace

void declare_config(LabelRemapper::Config& config) {
  using namespace config;
  name("ColormappedLabelsAdapter::Config");
  field<Path::Absolute>(config.remap_path, "remap_path");
  field(config.default_label, "default_label");
  check<Path::Exists>(config.remap_path, "remap_path");
}

LabelRemapper::LabelRemapper(const Config& config)
    : config(config::checkValid(config)) {
  const auto node = YAML::LoadFile(config.remap_path);
  const auto remappings = node.as<std::vector<LabelRemapRow>>();
  for (const auto& [sub, super] : remappings) {
    label_remapping_[sub] = super;
  }
}

void LabelRemapper::update(InputData& data) const {
  if (label_remapping_.empty()) {
    return;
  }

  // make sure data is right type before remapping
  if (data.label_image.type() != InputData::LabelMatType) {
    cv::Mat new_label_image(data.label_image.size(), InputData::LabelMatType);
    data.label_image.convertTo(new_label_image, InputData::LabelMatType);
    data.label_image = new_label_image;
  }

  auto& img = data.label_image;
  for (int r = 0; r < img.rows; ++r) {
    for (int c = 0; c < img.cols; ++c) {
      const auto old_label = img.at<InputData::LabelType>(r, c);
      auto iter = label_remapping_.find(old_label);
      // TODO(nathan) think about policy for keeping old label
      if (iter == label_remapping_.end()) {
        img.at<InputData::LabelType>(r, c) = config.default_label;
      } else {
        img.at<InputData::LabelType>(r, c) = iter->second;
      }
    }
  }
}

}  // namespace hydra
