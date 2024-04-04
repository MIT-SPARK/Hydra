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
#include "hydra/reconstruction/semantic_mesh_layer.h"

namespace hydra {

using voxblox::BlockIndex;
using voxblox::BlockIndexList;
using voxblox::FloatingPoint;
using voxblox::IndexSet;
using SemanticsPtr = std::vector<uint32_t>*;

SemanticMeshLayer::SemanticMeshLayer(FloatingPoint block_size)
    : mesh_(new voxblox::MeshLayer(block_size)), semantics_(new SemanticMeshMap()) {}

voxblox::Mesh::Ptr SemanticMeshLayer::allocateBlock(const BlockIndex& index,
                                                    bool use_semantics) {
  auto to_return = mesh_->allocateMeshPtrByIndex(index);

  if (use_semantics) {
    semantics_->emplace(index, std::vector<uint32_t>());
  }

  return to_return;
}

void SemanticMeshLayer::removeBlock(const BlockIndex& index) {
  mesh_->removeMesh(index);
  semantics_->erase(index);
}

voxblox::Mesh::Ptr SemanticMeshLayer::getMeshBlock(const BlockIndex& index) const {
  return mesh_->getMeshPtrByIndex(index);
}

SemanticsPtr SemanticMeshLayer::getSemanticBlock(const BlockIndex& index) const {
  auto iter = semantics_->find(index);
  if (iter == semantics_->end()) {
    return nullptr;
  }

  return &(iter->second);
}

void SemanticMeshLayer::getAllocatedBlockIndices(BlockIndexList& allocated) const {
  mesh_->getAllAllocatedMeshes(&allocated);
}

size_t SemanticMeshLayer::numVertices(bool only_active) const {
  BlockIndexList blocks;
  if (only_active) {
    mesh_->getAllUpdatedMeshes(&blocks);
  } else {
    mesh_->getAllAllocatedMeshes(&blocks);
  }

  size_t num_vertices = 0;
  for (const auto& idx : blocks) {
    const auto block = mesh_->getMeshPtrByIndex(idx);
    num_vertices += block->size();
  }

  return num_vertices;
}

size_t SemanticMeshLayer::numBlocks() const {
  return mesh_->getNumberOfAllocatedMeshes();
}

size_t SemanticMeshLayer::getMemorySize() const { return mesh_->getMemorySize(); }

SemanticMeshLayer::Ptr SemanticMeshLayer::clone() const {
  auto new_mesh = std::make_shared<SemanticMeshLayer>(mesh_->block_size());
  BlockIndexList all_indices;
  mesh_->getAllAllocatedMeshes(&all_indices);
  for (const auto& block_index : all_indices) {
    auto block = new_mesh->mesh_->allocateNewBlock(block_index);
    *block = *(mesh_->getMeshPtrByIndex(block_index));

    auto iter = semantics_->find(block_index);
    if (iter != semantics_->end()) {
      new_mesh->semantics_->emplace(iter->first, iter->second);
    }
  }

  return new_mesh;
}

void SemanticMeshLayer::merge(SemanticMeshLayer::Ptr& other) const {
  if (!other) {
    other.reset(new SemanticMeshLayer(mesh_->block_size()));
  }
}

void SemanticMeshLayer::merge(SemanticMeshLayer& other) const {
  CHECK_EQ(other.mesh_->block_size(), mesh_->block_size());

  BlockIndexList all_indices;
  mesh_->getAllAllocatedMeshes(&all_indices);
  for (const auto& block_index : all_indices) {
    auto block = other.mesh_->allocateNewBlock(block_index);
    *block = *(mesh_->getMeshPtrByIndex(block_index));

    auto iter = semantics_->find(block_index);
    if (iter != semantics_->end()) {
      auto oiter = other.semantics_->find(block_index);
      if (oiter != other.semantics_->end()) {
        oiter->second = iter->second;
      } else {
        other.semantics_->emplace(iter->first, iter->second);
      }
    }
  }
}

SemanticMeshLayer::Ptr SemanticMeshLayer::getActiveMesh(
    const BlockIndexList& archived) const {
  const IndexSet archived_set(archived.begin(), archived.end());
  auto active_mesh = std::make_shared<SemanticMeshLayer>(mesh_->block_size());

  BlockIndexList mesh_indices;
  mesh_->getAllUpdatedMeshes(&mesh_indices);
  for (const auto& block_index : mesh_indices) {
    if (archived_set.count(block_index)) {
      continue;
    }

    auto block = active_mesh->mesh_->allocateNewBlock(block_index);
    *block = *(mesh_->getMeshPtrByIndex(block_index));

    auto iter = semantics_->find(block_index);
    if (iter != semantics_->end()) {
      active_mesh->semantics_->emplace(iter->first, iter->second);
    }
  }

  return active_mesh;
}

void SemanticMeshLayer::pruneEmpty() {
  BlockIndexList mesh_indices;
  mesh_->getAllUpdatedMeshes(&mesh_indices);
  for (const auto& block_index : mesh_indices) {
    auto block = mesh_->getMeshPtrByIndex(block_index);
    if (!block->hasVertices()) {
      mesh_->removeMesh(block_index);
      semantics_->erase(block_index);
    }
  }
}

voxblox::MeshLayer::Ptr SemanticMeshLayer::getVoxbloxMesh() const { return mesh_; }

}  // namespace hydra
