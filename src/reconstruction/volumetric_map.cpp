// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#include "hydra/reconstruction/volumetric_map.h"

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/yaml.h>
#include <voxblox/io/layer_io.h>

#include "hydra/utils/display_utilities.h"

namespace voxblox {

using hydra::SemanticVoxel;

template <>
std::string getVoxelType<SemanticVoxel>() {
  return "semantic_voxel";
}

template <>
void mergeVoxelAIntoVoxelB(const SemanticVoxel& a, SemanticVoxel* b) {
  if (!b) {
    return;
  }

  if (a.empty) {
    return;
  }

  if (b->empty || b->semantic_likelihoods.rows() != a.semantic_likelihoods.rows()) {
    *b = a;
  }

  b->semantic_likelihoods += a.semantic_likelihoods;
  b->semantic_likelihoods.maxCoeff(&b->semantic_label);
}

template <>
void Block<SemanticVoxel>::serializeToIntegers(std::vector<uint32_t>* data) const {
  if (!data) {
    return;
  }

  // TODO(nathan) this is broken if endianness changes, but so is the rest of voxblox
  for (size_t i = 0; i < num_voxels_; ++i) {
    const auto& voxel = voxels_[i];
    data->push_back(voxel.semantic_label);
    if (voxel.empty) {
      data->push_back(0);
    } else {
      const uint32_t num_probs = voxel.semantic_likelihoods.rows();
      data->push_back(num_probs);
      // this is ugly, but less ugly than a bunch of deference reinterpret casts...
      const auto prev_size = data->size();
      data->resize(prev_size + num_probs);
      auto data_ptr = data->data() + prev_size;
      std::memcpy(data_ptr, voxel.semantic_likelihoods.data(), num_probs);
    }
  }
}

template <>
void Block<SemanticVoxel>::deserializeFromIntegers(const std::vector<uint32_t>& data) {
  size_t data_idx = 0;
  for (size_t i = 0; i < num_voxels_; ++i) {
    CHECK_LT(data_idx, data.size()) << "invalid serialization";
    auto& voxel = voxels_[i];
    voxel.semantic_label = data[data_idx];
    ++data_idx;

    const uint32_t num_probs = data[data_idx];
    ++data_idx;
    if (!num_probs) {
      voxel.empty = true;
      continue;
    }

    voxel.empty = false;
    CHECK_LE(data_idx + num_probs, data.size()) << " invalid serialization";
    voxel.semantic_likelihoods.resize(num_probs);
    const auto data_ptr = data.data() + data_idx;
    std::memcpy(voxel.semantic_likelihoods.data(), data_ptr, num_probs);
    data_idx += num_probs;
  }
}

}  // namespace voxblox

namespace hydra {

void declare_config(VolumetricMap::Config& config) {
  using namespace config;
  name("VolumetricMap");
  field(config.voxel_size, "voxel_size", "m");
  field(config.voxels_per_side, "voxels_per_side");
  field(config.truncation_distance, "truncation_distance", "m");

  check(config.voxel_size, GT, 0, "voxel_size");
  check(config.voxels_per_side, GT, 0, "voxels_per_side");
  check(config.truncation_distance, GT, 0, "truncation_distance");
}

VolumetricMap::VolumetricMap(const Config& _config,
                             bool with_semantics,
                             bool with_occupancy)
    : config(config::checkValid(_config)),
      tsdf_layer_(config.voxel_size, config.voxels_per_side),
      mesh_layer_(tsdf_layer_.block_size()),
      block_size(tsdf_layer_.block_size()),
      voxel_size_inv(tsdf_layer_.voxel_size_inv()) {
  if (with_semantics) {
    semantic_layer_.reset(new SemanticLayer(config.voxel_size, config.voxels_per_side));
  }

  if (with_occupancy) {
    occupancy_layer_.reset(
        new OccupancyLayer(config.voxel_size, config.voxels_per_side));
  }
}

void VolumetricMap::removeBlock(const voxblox::BlockIndex& block_index) {
  tsdf_layer_.removeBlock(block_index);
  mesh_layer_.removeBlock(block_index);
  if (semantic_layer_) {
    semantic_layer_->removeBlock(block_index);
  }

  if (occupancy_layer_) {
    occupancy_layer_->removeBlock(block_index);
  }
}

void VolumetricMap::save(const std::string& filepath) const {
  // TODO(nathan) consider hdf5 or something...
  YAML::Node config_node;
  config_node["map"] = config::toYaml(config);
  config_node["has_semantics"] = hasSemantics();
  config_node["has_occupancy"] = hasOccupancy();
  std::ofstream config_file(filepath + ".yaml");
  config_file << config_node;

  std::string map_path = filepath + ".vxblx";
  voxblox::io::SaveLayer(tsdf_layer_, map_path);
  if (semantic_layer_) {
    voxblox::io::SaveLayer(*semantic_layer_, map_path, false);
  }
}

std::unique_ptr<VolumetricMap> VolumetricMap::load(const std::string& filepath) {
  std::string map_path = filepath + ".vxblx";
  TsdfLayer::Ptr tsdf;
  if (!voxblox::io::LoadLayer<voxblox::TsdfVoxel>(map_path, true, &tsdf) || !tsdf) {
    return nullptr;
  }

  const auto cpath = filepath + ".yaml";
  const auto config = config::fromYamlFile<VolumetricMap::Config>(cpath, "map");
  if (std::abs(config.voxel_size - tsdf->voxel_size()) > 1.0e-5) {
    LOG(ERROR) << "TSDF voxel size does not match config voxel size";
    return nullptr;
  }

  if (static_cast<size_t>(config.voxels_per_side) != tsdf->voxels_per_side()) {
    LOG(ERROR) << "TSDF vps does not match config vps";
    return nullptr;
  }

  const auto node = YAML::LoadFile(cpath);
  bool use_semantics = false;
  if (node["has_semantics"] and node["has_semantics"].as<bool>()) {
    use_semantics = true;
  }

  bool use_occupancy;
  if (node["has_occupancy"] and node["has_occupancy"].as<bool>()) {
    use_occupancy = true;
  }

  auto map = std::make_unique<VolumetricMap>(config, use_semantics, use_occupancy);
  mergeLayer(*tsdf, map->tsdf_layer_, true);
  voxblox::io::LoadLayer<SemanticVoxel>(map_path, true, &map->semantic_layer_);
  return map;
}

std::string VolumetricMap::printStats() const {
  std::stringstream ss;
  const auto tsdf_size = tsdf_layer_.getMemorySize();
  ss << "TSDF: " << getHumanReadableMemoryString(tsdf_size);

  size_t semantic_size = 0;
  if (semantic_layer_) {
    semantic_size = semantic_layer_->getMemorySize();
    ss << ", SEMANTICS: " << getHumanReadableMemoryString(semantic_size);
  }

  size_t occ_size = 0;
  if (occupancy_layer_) {
    occ_size = occupancy_layer_->getMemorySize();
    ss << ", OCCUPANCY: " << getHumanReadableMemoryString(occ_size);
  }

  const auto mesh_size = mesh_layer_.getMemorySize();
  ss << ", MESH: " << getHumanReadableMemoryString(mesh_size);

  const auto total_size = tsdf_size + semantic_size + occ_size + mesh_size;
  ss << ", TOTAL: " << getHumanReadableMemoryString(total_size);

  return ss.str();
}

std::unique_ptr<VolumetricMap> VolumetricMap::fromTsdf(const TsdfLayer& tsdf,
                                                       double truncation_distance_m,
                                                       bool with_semantics,
                                                       bool with_occupancy) {
  VolumetricMap::Config config;
  config.voxel_size = tsdf.voxel_size();
  config.voxels_per_side = tsdf.voxels_per_side();
  config.truncation_distance = truncation_distance_m;
  auto to_return =
      std::make_unique<VolumetricMap>(config, with_semantics, with_occupancy);
  mergeLayer(tsdf, to_return->getTsdfLayer());
  return to_return;
}

std::unique_ptr<VolumetricMap> VolumetricMap::clone() const {
  const auto has_semantics = semantic_layer_ != nullptr;
  const auto has_occupancy = occupancy_layer_ != nullptr;
  auto map = std::make_unique<VolumetricMap>(config, has_semantics, has_occupancy);
  mergeLayer(tsdf_layer_, map->tsdf_layer_);
  mesh_layer_.merge(map->mesh_layer_);

  if (has_semantics) {
    mergeLayer(*semantic_layer_, *(map->semantic_layer_));
  }

  if (has_occupancy) {
    mergeLayer(*occupancy_layer_, *(map->occupancy_layer_));
  }

  return map;
}

void VolumetricMap::updateFrom(const VolumetricMap& other) {
  const auto has_semantics =
      semantic_layer_ != nullptr && other.semantic_layer_ != nullptr;
  const auto has_occupancy =
      occupancy_layer_ != nullptr && other.occupancy_layer_ != nullptr;

  mergeLayer(other.tsdf_layer_, tsdf_layer_);
  other.mesh_layer_.merge(mesh_layer_);

  if (has_semantics) {
    mergeLayer(*other.semantic_layer_, *semantic_layer_);
  }

  if (has_occupancy) {
    mergeLayer(*other.occupancy_layer_, *occupancy_layer_);
  }
}

}  // namespace hydra
