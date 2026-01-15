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
#include "hydra/backend/surface_place_utilities.h"

#include <glog/logging.h>

namespace hydra::utils {

using spark_dsg::Place2dNodeAttributes;

void reallocateMeshPoints(const spark_dsg::Mesh& mesh,
                          Place2dNodeAttributes& attrs1,
                          Place2dNodeAttributes& attrs2) {
  Eigen::Vector2d delta = attrs2.position.head(2) - attrs1.position.head(2);
  Eigen::Vector2d d = attrs1.position.head(2) + delta / 2;

  std::vector<size_t> p1_new_indices;
  std::vector<size_t> p2_new_indices;

  for (auto midx : attrs1.pcl_mesh_connections) {
    Eigen::Vector2d p = mesh.points.at(midx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }

  for (auto midx : attrs2.pcl_mesh_connections) {
    Eigen::Vector2d p = mesh.points.at(midx).head(2).cast<double>();
    if ((p - d).dot(delta) > 0) {
      p2_new_indices.push_back(midx);
    } else {
      p1_new_indices.push_back(midx);
    }
  }

  if (p1_new_indices.size() == 0 || p2_new_indices.size() == 0) {
    LOG(ERROR) << "Reallocating mesh points would make empty place. Skippings.";
    return;
  }

  std::sort(p1_new_indices.begin(), p1_new_indices.end());
  auto last = std::unique(p1_new_indices.begin(), p1_new_indices.end());
  p1_new_indices.erase(last, p1_new_indices.end());

  std::sort(p2_new_indices.begin(), p2_new_indices.end());
  last = std::unique(p2_new_indices.begin(), p2_new_indices.end());
  p2_new_indices.erase(last, p2_new_indices.end());

  attrs1.pcl_mesh_connections = p1_new_indices;
  attrs2.pcl_mesh_connections = p2_new_indices;

  // Say there are active mesh indices if either involved node has them.
  // In theory we could actually check if any of the reallocated vertices changes a
  // place's activeness for a small speed improvement, but not sure how much it
  // matters
  attrs1.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
  attrs2.has_active_mesh_indices =
      attrs1.has_active_mesh_indices || attrs2.has_active_mesh_indices;
}

}  // namespace hydra::utils
