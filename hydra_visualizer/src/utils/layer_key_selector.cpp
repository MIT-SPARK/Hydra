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
#include "hydra_visualizer/utils/layer_key_selector.h"

#include <glog/logging.h>

#include <regex>

namespace hydra {

using namespace spark_dsg;

std::optional<LayerKeySelector> LayerKeySelector::parse(const std::string& str) {
  std::regex re(R"((\d+)p\*$|(\d+)\*$|(\d+)p(\d+)$|(\d+)$)");
  std::smatch match;
  if (!std::regex_match(str, match, re)) {
    return std::nullopt;
  }

  CHECK_EQ(match.size(), 6);
  if (!match.str(1).empty()) {
    // partitions >= 1
    return LayerKeySelector{LayerKey{std::stol(match.str(1))}, true, false};
  } else if (!match.str(2).empty()) {
    // partitions >= 0
    return LayerKeySelector{LayerKey{std::stol(match.str(2))}, true, true};
  } else if (!match.str(3).empty()) {
    // layer and partition
    const auto part_id = static_cast<PartitionId>(std::stoi(match.str(4)));
    return LayerKeySelector{LayerKey{std::stol(match.str(3)), part_id}};
  } else {
    // layer and partition 0
    return LayerKeySelector{LayerKey{std::stol(match.str(5))}};
  }
}

std::string LayerKeySelector::str() const {
  const auto layer_str = std::to_string(key.layer);
  if (wildcard) {
    return layer_str + (include_default ? "*" : "p*");
  }

  if (key.partition) {
    return layer_str + "p" + std::to_string(key.partition);
  }

  return layer_str;
}

bool LayerKeySelector::matches(LayerKey to_match) const {
  if (key.layer != to_match.layer) {
    return false;
  }

  if (wildcard) {
    return include_default || to_match.partition;
  }

  return key.partition == to_match.partition;
}

bool LayerKeySelector::operator<(const LayerKeySelector& other) const {
  if (key < other.key) {
    return true;
  }

  if (key == other.key) {
    return wildcard < other.wildcard;
  } else {
    return false;
  }
}

bool LayerKeySelector::operator==(const LayerKeySelector& other) const = default;

bool LayerKeySelector::operator!=(const LayerKeySelector& other) const = default;

std::string SelectorConversion::toIntermediate(const LayerKeySelector& value,
                                               std::string&) {
  return value.str();
}

void SelectorConversion::fromIntermediate(const std::string& intermediate,
                                          LayerKeySelector& value,
                                          std::string& error) {
  const auto parsed = LayerKeySelector::parse(intermediate);
  if (!parsed) {
    error = "Invalid layer selector '" + intermediate + "', must be of form " +
            "integer layer (e.g., '3')" + ", layer and partition (e.g., '2p1')" +
            ", or layer and wildcard (e.g., '4p*')";
    return;
  }

  value = *parsed;
}

std::string LayerKeyConversion::toIntermediate(const LayerKey& value, std::string&) {
  return LayerKeySelector{value}.str();
}

void LayerKeyConversion::fromIntermediate(const std::string& intermediate,
                                          LayerKey& value,
                                          std::string& error) {
  std::regex re(R"((\d+)p(\d+)$|(\d+)$)");
  std::smatch match;
  if (!std::regex_match(intermediate, match, re)) {
    error = "Invalid layer key '" + intermediate + "'!";
    return;
  }

  CHECK_EQ(match.size(), 4);
  if (!match.str(1).empty()) {
    // layer and partition
    const auto part_id = static_cast<PartitionId>(std::stoi(match.str(2)));
    value = LayerKey{std::stol(match.str(1)), part_id};
  } else {
    value = LayerKey{std::stol(match.str(3))};
  }
}

}  // namespace hydra
