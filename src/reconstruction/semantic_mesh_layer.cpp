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

size_t SemanticMeshLayer::numBlocks() const {
  return mesh_->getNumberOfAllocatedMeshes();
}

size_t SemanticMeshLayer::getMemorySize() const { return mesh_->getMemorySize(); }

SemanticMeshLayer::Ptr SemanticMeshLayer::getActiveMesh(const IndexSet& archived) {
  auto active_mesh = std::make_shared<SemanticMeshLayer>(mesh_->block_size());

  BlockIndexList mesh_indices;
  mesh_->getAllUpdatedMeshes(&mesh_indices);
  for (const auto& block_index : mesh_indices) {
    if (archived.count(block_index)) {
      continue;
    }

    auto block = active_mesh->mesh_->allocateNewBlock(block_index);
    *block = *(mesh_->getMeshPtrByIndex(block_index));

    auto iter = semantics_->find(block_index);
    if (iter != semantics_->end()) {
      active_mesh->semantics_->emplace(iter->first, iter->second);
    }
  }

  // TODO(nathan) maybe only do this for archived blocks?
  for (const auto& block_index : mesh_indices) {
    auto block = mesh_->getMeshPtrByIndex(block_index);
    if (!block->hasVertices()) {
      mesh_->removeMesh(block_index);
      semantics_->erase(block_index);
    }
  }

  return active_mesh;
}

voxblox::MeshLayer::Ptr SemanticMeshLayer::getVoxbloxMesh() const { return mesh_; }

kimera_pgmo::SemanticVoxbloxMeshInterface SemanticMeshLayer::getMeshInterface() const {
  return kimera_pgmo::SemanticVoxbloxMeshInterface(mesh_, semantics_);
}

}  // namespace hydra
