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
#include "hydra/utils/pgmo_mesh_traits.h"

namespace spark_dsg {

size_t pgmoNumVertices(const Mesh& mesh) { return mesh.numVertices(); }

void pgmoResizeVertices(Mesh& mesh, size_t size) { mesh.resizeVertices(size); }

Eigen::Vector3f pgmoGetVertex(const Mesh& mesh,
                              size_t i,
                              kimera_pgmo::traits::VertexTraits* traits) {
  if (!traits) {
    return mesh.pos(i);
  }

  if (mesh.has_colors) {
    const auto c = mesh.color(i);
    traits->color = {{c.r, c.g, c.b, c.a}};
  }

  if (mesh.has_timestamps) {
    traits->stamp = mesh.timestamp(i);
  }

  if (mesh.has_labels) {
    traits->label = mesh.label(i);
  }

  return mesh.pos(i);
}

void pgmoSetVertex(Mesh& mesh,
                   size_t i,
                   const Eigen::Vector3f& pos,
                   const kimera_pgmo::traits::VertexTraits& traits) {
  mesh.setPos(i, pos);
  if (traits.color && mesh.has_colors) {
    const auto& c = *traits.color;
    mesh.setColor(i, Color(c[0], c[1], c[2], c[3]));
  }

  if (traits.stamp && mesh.has_timestamps) {
    mesh.setTimestamp(i, *traits.stamp);
  }

  if (traits.label && mesh.has_labels) {
    mesh.setLabel(i, *traits.label);
  }
}

uint64_t pgmoGetVertexStamp(const Mesh& mesh, size_t i) {
  if (!mesh.has_timestamps) {
    throw std::runtime_error("mesh has no timestamps");
  }

  return mesh.stamps.at(i);
}

size_t pgmoNumFaces(const Mesh& mesh) { return mesh.numFaces(); }

void pgmoResizeFaces(Mesh& mesh, size_t size) { mesh.resizeFaces(size); }

kimera_pgmo::traits::Face pgmoGetFace(const Mesh& mesh, size_t i) {
  return mesh.face(i);
}

void pgmoSetFace(Mesh& mesh, size_t i, const kimera_pgmo::traits::Face& face) {
  mesh.face(i) = face;
}

}  // namespace spark_dsg
