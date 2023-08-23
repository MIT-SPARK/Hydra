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
#include <kimera_pgmo/utils/VoxbloxMeshInterface.h>
#include <voxblox/mesh/mesh_layer.h>

#include <memory>

namespace hydra {

class SemanticMeshLayer {
 public:
  using Ptr = std::shared_ptr<SemanticMeshLayer>;
  using SemanticMeshMap = voxblox::AnyIndexHashMapType<std::vector<uint32_t>>::type;

  explicit SemanticMeshLayer(voxblox::FloatingPoint block_size);

  voxblox::Mesh::Ptr allocateBlock(const voxblox::BlockIndex& index,
                                   bool use_semantics);

  void removeBlock(const voxblox::BlockIndex& index);

  voxblox::Mesh::Ptr getMeshBlock(const voxblox::BlockIndex& index) const;

  std::vector<uint32_t>* getSemanticBlock(const voxblox::BlockIndex& index) const;

  void getAllocatedBlockIndices(voxblox::BlockIndexList& allocated) const;

  size_t numBlocks() const;

  size_t getMemorySize() const;

  SemanticMeshLayer::Ptr clone() const;

  void merge(SemanticMeshLayer::Ptr& other) const;

  SemanticMeshLayer::Ptr getActiveMesh(
      const voxblox::BlockIndexList& archived_blocks) const;

  void pruneEmpty();

  voxblox::MeshLayer::Ptr getVoxbloxMesh() const;

  kimera_pgmo::SemanticVoxbloxMeshInterface getMeshInterface() const;

 protected:
  voxblox::MeshLayer::Ptr mesh_;
  std::shared_ptr<SemanticMeshMap> semantics_;
};

}  // namespace hydra
