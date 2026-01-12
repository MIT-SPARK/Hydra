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

#include <kimera_pgmo/mesh_types.h>
#include <spark_dsg/mesh.h>

#include "hydra/reconstruction/voxel_types.h"

namespace spark_dsg {

size_t pgmoNumVertices(const Mesh& mesh);

void pgmoResizeVertices(Mesh& mesh, size_t size);

kimera_pgmo::traits::VertexProperties pgmoGetVertexProperties(const Mesh& mesh);

Eigen::Vector3f pgmoGetVertex(const Mesh& mesh,
                              size_t i,
                              kimera_pgmo::traits::VertexTraits* traits = nullptr);

void pgmoSetVertex(Mesh& mesh,
                   size_t i,
                   const Eigen::Vector3f& pos,
                   const kimera_pgmo::traits::VertexTraits* traits = nullptr);

uint64_t pgmoGetVertexStamp(const Mesh& mesh, size_t i);

size_t pgmoNumFaces(const Mesh& mesh);

void pgmoResizeFaces(Mesh& mesh, size_t size);

kimera_pgmo::traits::Face pgmoGetFace(const Mesh& mesh, size_t i);

void pgmoSetFace(Mesh& mesh, size_t i, const kimera_pgmo::traits::Face& face);

}  // namespace spark_dsg

namespace hydra {

struct BlockMeshIter {
  explicit BlockMeshIter(const MeshLayer& mesh) : mesh(mesh) {}

  struct const_iterator {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = const std::pair<BlockIndex, const spark_dsg::Mesh&>;

    explicit const_iterator(MeshLayer::const_iterator iter) : iter_(iter) {}

    value_type operator*() const { return {iter_->index, *iter_}; }

    const_iterator& operator++() {
      ++iter_;
      return *this;
    }

    const_iterator operator++(int) {
      auto tmp = *this;
      ++iter_;
      return tmp;
    }

    bool operator==(const const_iterator& other) const { return other.iter_ == iter_; }

    bool operator!=(const const_iterator& other) const { return other.iter_ != iter_; }

   private:
    MeshLayer::const_iterator iter_;
  };

  const MeshLayer& mesh;
  const_iterator begin() const { return const_iterator(mesh.begin()); }
  const_iterator end() const { return const_iterator(mesh.end()); }
};

}  // namespace hydra
