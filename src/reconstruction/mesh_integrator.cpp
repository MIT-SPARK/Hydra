// The original implementation from https://github.com/personalrobotics/OpenChisel and
// subsequent modifications falls under the following license:
//
// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Modifications are also subject to the following copyright and disclaimer:
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
#include "hydra/reconstruction/mesh_integrator.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spatial_hash/hash.h>

#include <iomanip>
#include <list>
#include <thread>

#include "hydra/common/global_info.h"
#include "hydra/reconstruction/marching_cubes.h"
#include "hydra/reconstruction/volumetric_map.h"
#include "hydra/utils/misc.h"

namespace hydra {

using spatial_hash::LongIndex;
using spatial_hash::LongIndexHashMap;

namespace {

using IndexOffsets = Eigen::Matrix<int, 3, 8>;
using CoordOffsets = Eigen::Matrix<float, 3, 8>;

template <typename Scalar>
std::string indexToStr(const Eigen::Matrix<Scalar, 3, 1>& idx) {
  return "(" + std::to_string(idx.x()) + ", " + std::to_string(idx.y()) + ", " +
         std::to_string(idx.z()) + ")";
}

template <typename Block>
const Block* maybeGetBlockPtr(const spatial_hash::BlockLayer<Block>* layer,
                              const BlockIndex& index) {
  if (!layer) {
    return nullptr;
  }

  return layer->getBlockPtr(index).get();
}

struct VoxelInfo {
  const TsdfVoxel& tsdf;
  const SemanticVoxel* semantics;
  const TrackingVoxel* tracking;
};

struct BlockInfo {
  BlockInfo() = default;
  explicit BlockInfo(const VolumetricMap& map, const BlockIndex& index)
      : index(index),
        tracking(maybeGetBlockPtr(map.getTrackingLayer(), index)),
        semantics(maybeGetBlockPtr(map.getSemanticLayer(), index)),
        tsdf_(map.getTsdfLayer().getBlockPtr(index).get()) {}

  BlockIndex index;
  const TrackingBlock* tracking;
  const SemanticBlock* semantics;

  operator bool() const { return tsdf_ != nullptr; }

  const TsdfBlock& tsdf() const { return *tsdf_; }

  VoxelInfo voxel(const VoxelIndex& index) const {
    return {tsdf_->getVoxel(index),
            semantics ? &semantics->getVoxel(index) : nullptr,
            tracking ? &tracking->getVoxel(index) : nullptr};
  }

 private:
  const TsdfBlock* tsdf_;
};

inline BlockIndex neighborIndex(const BlockIndex& block_idx,
                                int voxels_per_side,
                                VoxelIndex& corner_index) {
  BlockIndex block_offset = BlockIndex::Zero();
  for (unsigned int j = 0u; j < 3u; j++) {
    if (corner_index(j) < 0) {
      block_offset(j) = -1;
      corner_index(j) = corner_index(j) + voxels_per_side;
    } else if (corner_index(j) >= static_cast<BlockIndex::Scalar>(voxels_per_side)) {
      block_offset(j) = 1;
      corner_index(j) = corner_index(j) - voxels_per_side;
    }
  }

  return block_idx + block_offset;
}

bool fillPoints(const VolumetricMap& map,
                const IndexOffsets& index_offsets,
                const CoordOffsets& coord_offsets,
                const BlockInfo& block,
                const VoxelIndex& index,
                float min_weight,
                MarchingCubes::SdfPoints& points) {
  const auto coords = block.tsdf().getVoxelPosition(index);
  for (int i = 0; i < 8; ++i) {
    auto& point = points[i];
    VoxelIndex c_idx = index + index_offsets.col(i);

    BlockInfo v_block;
    if (block.tsdf().isValidVoxelIndex(c_idx)) {
      v_block = block;
    } else {
      // we have to access a different block
      const auto n_idx = neighborIndex(block.index, map.config.voxels_per_side, c_idx);
      v_block = BlockInfo(map, n_idx);
    }

    if (!v_block) {
      return false;
    }

    const auto voxel = v_block.voxel(c_idx);
    if (voxel.tsdf.weight < min_weight) {
      return false;
    }

    point.distance = voxel.tsdf.distance;
    point.weight = voxel.tsdf.weight;
    point.pos = coords + coord_offsets.col(i);
    point.color = voxel.tsdf.color;
    point.tracking_voxel = voxel.tracking;
    if (voxel.semantics && !voxel.semantics->empty) {
      point.label = voxel.semantics->semantic_label;
    }
  }

  return true;
}

void meshBlock(const IndexOffsets& idx_offsets,
               const CoordOffsets& pos_offsets,
               const BlockIndex& block_index,
               float w_min,
               VolumetricMap& map) {
  BlockInfo block(map, block_index);
  auto mesh = map.getMeshLayer().getBlockPtr(block_index);
  if (!block || !mesh) {
    LOG(ERROR) << "Invalid block index: " << block_index.transpose();
    return;
  }

  VoxelIndex idx;
  MarchingCubes::SdfPoints points;
  const int vps = map.config.voxels_per_side;
  for (idx.x() = 0; idx.x() < vps; ++idx.x()) {
    for (idx.y() = 0; idx.y() < vps; ++idx.y()) {
      for (idx.z() = 0; idx.z() < vps; ++idx.z()) {
        bool interior = idx.x() < vps && idx.y() < vps && idx.z() < vps;
        VLOG(15) << "[mesh] processing " << (interior ? "interior" : "exterior")
                 << "voxel: " << indexToStr(idx);

        if (!fillPoints(map, idx_offsets, pos_offsets, block, idx, w_min, points)) {
          continue;
        }

        ::hydra::MarchingCubes::meshCube(points, *mesh);
      }
    }
  }
}

// TODO(nathan) fix
void showUpdateInfo(const VolumetricMap& map,
                    const BlockIndices& blocks,
                    int verbosity) {
  if (!VLOG_IS_ON(verbosity)) {
    return;
  }

  const auto& mesh_layer = map.getMeshLayer();
  std::stringstream ss;
  ss << "Updated blocks:";
  for (const auto& idx : blocks) {
    const auto& block = mesh_layer.getBlock(idx);
    ss << "  - " << std::setw(4) << std::setfill(' ') << block.points.size()
       << " vertices @ " << indexToStr(idx);
  }

  VLOG(10) << ss.str();
}

}  // namespace

MeshIntegrator::Config::Config()
    : integrator_threads(GlobalInfo::instance().getConfig().default_num_threads) {}

void declare_config(MeshIntegrator::Config& config) {
  using namespace config;
  name("MeshIntegrator::Config");
  field(config.min_weight, "min_weight");
  field<ThreadNumConversion>(config.integrator_threads, "integrator_threads");
  field(config.flatten_blocks, "flatten_blocks");
  field(config.vertex_tolerance_m, "vertex_tolerance_m");

  check(config.min_weight, GT, 0.0f, "min_weight");
  check(config.integrator_threads, GT, 0, "integrator_threads");
  check(config.vertex_tolerance_m, GT, 0.0f, "vertex_tolerance_m");
}

const Eigen::Matrix<int, 3, 8> MeshIntegrator::cube_index_offsets_ = [] {
  Eigen::Matrix<int, 3, 8> offsets;
  // clang-format off
  offsets << 0, 1, 1, 0, 0, 1, 1, 0,
             0, 0, 1, 1, 0, 0, 1, 1,
             0, 0, 0, 0, 1, 1, 1, 1;
  // clang-format on
  return offsets;
}();

MeshIntegrator::MeshIntegrator(const Config& config)
    : config(config::checkValid(config)) {}

void MeshIntegrator::allocateBlocks(const BlockIndices& blocks,
                                    VolumetricMap& map) const {
  auto& mesh_layer = map.getMeshLayer();
  for (const BlockIndex& block_index : blocks) {
    auto& mesh =
        mesh_layer.allocateBlock(block_index, map.hasSemantics(), map.hasTracking());
    mesh.clear();
  }
}

void MeshIntegrator::generateMesh(VolumetricMap& map,
                                  bool only_mesh_updated_blocks,
                                  bool clear_updated_flag) const {
  // TODO(nathan) think about this more
  cube_coord_offsets_ = cube_index_offsets_.cast<float>() * map.config.voxel_size;
  const auto& tsdf = map.getTsdfLayer();
  const BlockIndices blocks =
      only_mesh_updated_blocks ? tsdf.blockIndicesWithCondition(TsdfBlock::meshUpdated)
                               : tsdf.allocatedBlockIndices();

  allocateBlocks(blocks, map);
  launchThreads(blocks, map);
  showUpdateInfo(map, blocks, 5);
  for (const auto& block_idx : blocks) {
    map.getMeshLayer().getBlock(block_idx).updated = true;
  }

  if (!clear_updated_flag) {
    return;
  }

  for (const auto& block_idx : blocks) {
    tsdf.getBlock(block_idx).mesh_updated = false;
  }
}

void MeshIntegrator::launchThreads(const BlockIndices& blocks,
                                   VolumetricMap& map) const {
  BlockIndexGetter index_getter(blocks);
  std::list<std::thread> threads;
  for (int i = 0; i < config.integrator_threads; ++i) {
    threads.emplace_back(&MeshIntegrator::processBlock, this, &map, &index_getter);
  }

  for (std::thread& thread : threads) {
    thread.join();
  }
}

void MeshIntegrator::processBlock(VolumetricMap* map,
                                  BlockIndexGetter* index_getter) const {
  BlockIndex b_idx;
  while (index_getter->getNextIndex(b_idx)) {
    VLOG(10) << "Extracting mesh for block: " << indexToStr(b_idx);
    meshBlock(cube_index_offsets_, cube_coord_offsets_, b_idx, config.min_weight, *map);
    if (!config.flatten_blocks) {
      continue;
    }

    auto mesh = map->getMeshLayer().getBlockPtr(b_idx);
    if (!mesh) {
      continue;
    }

    const auto scale = 1.0f / config.vertex_tolerance_m;

    size_t new_idx = 0;
    LongIndexHashMap<size_t> seen;
    std::vector<size_t> remap(mesh->points.size(), 0);
    std::vector<size_t> to_delete;
    for (size_t i = 0; i < mesh->points.size(); ++i) {
      const auto& p = mesh->points[i];
      const LongIndex p_idx(std::round(p.x() * scale),
                            std::round(p.y() * scale),
                            std::round(p.z() * scale));
      auto iter = seen.find(p_idx);
      if (iter != seen.end()) {
        remap[i] = iter->second;
        to_delete.push_back(i);
      } else {
        remap[i] = new_idx;
        seen[p_idx] = new_idx;
        ++new_idx;
      }
    }

    for (auto& face : mesh->faces) {
      face[0] = remap[face[0]];
      face[1] = remap[face[1]];
      face[2] = remap[face[2]];
    }

    utils::remove_by_index(mesh->points, to_delete);
    if (mesh->has_colors) {
      utils::remove_by_index(mesh->colors, to_delete);
    }

    if (mesh->has_timestamps) {
      utils::remove_by_index(mesh->stamps, to_delete);
    }

    if (mesh->has_first_seen_stamps) {
      utils::remove_by_index(mesh->first_seen_stamps, to_delete);
    }

    if (mesh->has_labels) {
      utils::remove_by_index(mesh->labels, to_delete);
    }
  }
}

}  // namespace hydra
