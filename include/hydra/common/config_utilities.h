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
#include <spark_dsg/scene_graph_types.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <map>
#include <string>

#include "hydra/common/common_types.h"

namespace hydra {

template <typename T>
struct LayerMapConversion {
  using TargetMap = std::map<std::string, T>;
  using SourceMap = std::map<spark_dsg::LayerId, T>;

  static TargetMap toIntermediate(const SourceMap& other, std::string&) {
    TargetMap to_return;
    for (const auto& kv_pair : other) {
      to_return[spark_dsg::DsgLayers::LayerIdToString(kv_pair.first)] = kv_pair.second;
    }

    return to_return;
  }

  static void fromIntermediate(const TargetMap& other, SourceMap& value, std::string&) {
    value.clear();
    for (const auto& kv_pair : other) {
      value[spark_dsg::DsgLayers::StringToLayerId(kv_pair.first)] = kv_pair.second;
    }
  }
};

struct QuaternionConverter {
  using DoubleMap = std::map<std::string, double>;

  static DoubleMap toIntermediate(const Eigen::Quaterniond& other, std::string& error);

  static void fromIntermediate(const DoubleMap& other,
                               Eigen::Quaterniond& value,
                               std::string& error);
};

}  // namespace hydra

namespace YAML {

template <>
struct convert<spark_dsg::Color> {
  static Node encode(const spark_dsg::Color& rhs) {
    auto node = YAML::Node(YAML::NodeType::Sequence);
    node.push_back(static_cast<int>(rhs.r));
    node.push_back(static_cast<uint8_t>(rhs.g));
    node.push_back(static_cast<uint8_t>(rhs.b));
    node.push_back(static_cast<uint8_t>(rhs.a));
    return node;
  }

  static bool decode(const Node& node, spark_dsg::Color& rhs) {
    if (!node.IsSequence()) {
      throw std::runtime_error("Color must be a sequence!");
    }
    if (node.size() < 3 || node.size() > 4) {
      throw std::runtime_error("Invalid color representation with '" +
                               std::to_string(node.size()) + "' channels!");
    }

    rhs.r = static_cast<uint8_t>(node[0].as<int>());
    rhs.g = static_cast<uint8_t>(node[1].as<int>());
    rhs.b = static_cast<uint8_t>(node[2].as<int>());
    rhs.a = node.size() == 4 ? static_cast<uint8_t>(node[3].as<int>()) : 255;
    return true;
  }
};

}  // namespace YAML
