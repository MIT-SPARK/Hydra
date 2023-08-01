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
#pragma once
#include <array>
#include <atomic>
#include <iostream>
#include <map>
#include <memory>
#include <vector>

#include "hydra/common/label_space_config.h"

namespace kimera {
class SemanticColorMap;
}

namespace hydra {

class HydraConfig {
 public:
  using ColorArray = std::array<uint8_t, 3>;
  using LabelNameMap = std::map<uint8_t, std::string>;

  static HydraConfig& instance();

  bool force_shutdown() const;

  void setForceShutdown(bool force_shutdown);

  void setRoomColorMap(const std::vector<ColorArray>& colormap);

  const ColorArray& getRoomColor(size_t index) const;

  const LabelNameMap& getLabelToNameMap() const;

  void setLabelToNameMap(const LabelNameMap& name_map);

  void setLabelSpaceConfig(const LabelSpaceConfig& config);

  const LabelSpaceConfig& getLabelSpaceConfig() const;

  size_t getTotalLabels() const;

  // this intentionally returns a shared ptr to be threadsafe
  std::shared_ptr<kimera::SemanticColorMap> setRandomColormap();

  // this intentionally returns a shared ptr to be threadsafe
  std::shared_ptr<kimera::SemanticColorMap> getSemanticColorMap() const;

  // this invalidates any instances (mostly intended for testing)
  static void reset();

 private:
  HydraConfig();

  static std::unique_ptr<HydraConfig> instance_;

  // TODO(nathan) consider moving robot id and logging here
  std::atomic<bool> force_shutdown_;

  std::vector<ColorArray> room_colormap_;

  LabelSpaceConfig label_space_;
  std::map<uint8_t, std::string> label_to_name_map_;
  std::shared_ptr<kimera::SemanticColorMap> label_colormap_;
};

std::ostream& operator<<(std::ostream& out, const HydraConfig& config);

}  // namespace hydra
