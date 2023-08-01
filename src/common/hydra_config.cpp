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
#include "hydra/common/hydra_config.h"

#include <kimera_semantics/color.h>

namespace hydra {

using ColorMapPtr = std::shared_ptr<kimera::SemanticColorMap>;
decltype(HydraConfig::instance_) HydraConfig::instance_;

HydraConfig& HydraConfig::instance() {
  if (!instance_) {
    instance_.reset(new HydraConfig());
  }
  return *instance_;
}

std::ostream& operator<<(std::ostream& out, const HydraConfig& config) {
  out << "{force_shutdown: " << std::boolalpha << config.force_shutdown() << "}";
  return out;
}

HydraConfig::HydraConfig() : force_shutdown_(false) {
  room_colormap_ = {
      {166, 206, 227},
      {31, 120, 180},
      {178, 223, 138},
      {51, 160, 44},
      {251, 154, 153},
      {227, 26, 28},
      {253, 191, 111},
      {255, 127, 0},
      {202, 178, 214},
      {106, 61, 154},
      {255, 255, 153},
      {177, 89, 40},
  };

  label_colormap_.reset(new kimera::SemanticColorMap());
}

bool HydraConfig::force_shutdown() const { return force_shutdown_; }

void HydraConfig::setForceShutdown(bool force_shutdown) {
  force_shutdown_ = force_shutdown;
}

void HydraConfig::setRoomColorMap(const std::vector<ColorArray>& colormap) {
  room_colormap_ = colormap;
}

const HydraConfig::ColorArray& HydraConfig::getRoomColor(size_t index) const {
  return room_colormap_.at(index % room_colormap_.size());
}

void HydraConfig::setLabelToNameMap(const HydraConfig::LabelNameMap& name_map) {
  label_to_name_map_ = name_map;
}

const HydraConfig::LabelNameMap& HydraConfig::getLabelToNameMap() const {
  return label_to_name_map_;
}

void HydraConfig::setLabelSpaceConfig(const LabelSpaceConfig& config) {
  label_space_ = config;
  if (!label_space_.colormap_filepath.empty()) {
    label_colormap_ =
        kimera::SemanticColorMap::fromFile(label_space_.colormap_filepath);
  } else {
    label_colormap_ = kimera::SemanticColorMap::randomColors(label_space_.total_labels);
  }

  if (label_colormap_) {
    VLOG(1) << "Loaded label space colors:" << std::endl << *label_colormap_;
  }
}

ColorMapPtr HydraConfig::setRandomColormap() {
  label_colormap_ = kimera::SemanticColorMap::randomColors(label_space_.total_labels);
  return label_colormap_;
}

const LabelSpaceConfig& HydraConfig::getLabelSpaceConfig() const {
  return label_space_;
}

size_t HydraConfig::getTotalLabels() const { return label_space_.total_labels; }

ColorMapPtr HydraConfig::getSemanticColorMap() const { return label_colormap_; }

void HydraConfig::reset() { instance_.reset(new HydraConfig()); }

}  // namespace hydra
