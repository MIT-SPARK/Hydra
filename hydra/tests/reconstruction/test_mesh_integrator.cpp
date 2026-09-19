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
#include <gtest/gtest.h>
#include <hydra/reconstruction/mesh_integrator.h>
#include <hydra/reconstruction/volumetric_map.h>

namespace hydra {
namespace {

void fillBlock(VolumetricMap& map, int x, int y, int z, size_t axis) {
  const spark_dsg::Color black(0, 0, 0);
  const spark_dsg::Color grey(100, 100, 100);

  const BlockIndex index(x, y, z);
  map.allocateBlock(index);
  auto block = map.getBlock(index);
  for (size_t i = 0; i < block.tsdf->numVoxels(); ++i) {
    auto& voxel = block.tsdf->getVoxel(i);
    voxel.distance = block.tsdf->getVoxelPosition(i)[axis] + 1.0f;
    voxel.weight = 1.0f;
    voxel.color = voxel.distance < 0 ? black : grey;
    auto& semantic = block.semantic->getVoxel(i);
    semantic.empty = false;
    semantic.semantic_label = voxel.distance < 0 ? 1 : 2;
    auto& tracking = block.tracking->getVoxel(i);
    tracking.first_observed = voxel.distance < 0 ? 10 : 20;
    tracking.last_observed = voxel.distance < 0 ? 30 : 40;
  }
}

void meshBlockNoCache(VolumetricMap& map,
                      const BlockIndex& index,
                      const MeshIntegrator& integrator) {
  auto& to_mesh = map.getMeshLayer().getBlock(index);
  to_mesh.clear();
  integrator.meshBlockInterior(index, VoxelIndex::Zero(), map);
  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        if (x || y || z) {
          integrator.meshBlockExterior(index, VoxelIndex(x, y, z), map);
        }
      }
    }
  }
}

bool faceSame(const spark_dsg::Mesh& lhs,
              const spark_dsg::Mesh& rhs,
              size_t lhs_face,
              size_t rhs_face) {
  for (size_t k = 0; k < 3; ++k) {
    if (!lhs.pos(lhs.faces[lhs_face][k]).isApprox(rhs.pos(rhs.faces[rhs_face][k]))) {
      return false;
    }
  }

  return true;
}

}  // namespace

TEST(MeshIntegrator, FacesWithVertexCacheCorrect) {
  VolumetricMap::Config map_config;
  map_config.voxel_size = 1.0f;
  map_config.voxels_per_side = 2;
  map_config.with_semantics = true;
  map_config.with_tracking = true;

  MeshIntegrator::Config config;
  config.integrator_threads = 1;
  const MeshIntegrator integrator(config);

  for (int axis = 0; axis < 3; ++axis) {
    SCOPED_TRACE(axis);
    VolumetricMap map(map_config);
    for (int x = -1; x <= 0; ++x) {
      for (int y = -1; y <= 0; ++y) {
        for (int z = -1; z <= 0; ++z) {
          fillBlock(map, x, y, z, axis);
        }
      }
    }

    integrator.generateMesh(map, false, false);

    const BlockIndex index(-1, -1, -1);
    const MeshBlock with_cache = map.getMeshLayer().getBlock(index);
    ASSERT_EQ(with_cache.numVertices(), 9u);
    ASSERT_EQ(with_cache.numFaces(), 8u);
    ASSERT_EQ(with_cache.face_voxels.size(), 8u);
    for (size_t i = 0; i < with_cache.numVertices(); ++i) {
      EXPECT_EQ(with_cache.labels[i], 2u);
      EXPECT_EQ(with_cache.colors[i], spark_dsg::Color(50, 50, 50));
      EXPECT_EQ(with_cache.first_seen_stamps[i], 10u);
      EXPECT_EQ(with_cache.stamps[i], 40u);
    }

    // Reconstruct the same cells without a cache to compare winding, attributes,
    // and face provenance independently of the vertex numbering.
    meshBlockNoCache(map, index, integrator);

    auto& no_cache = map.getMeshLayer().getBlock(index);
    ASSERT_EQ(no_cache.numFaces(), with_cache.numFaces());
    for (size_t i = 0; i < with_cache.numFaces(); ++i) {
      size_t matches = 0;
      for (size_t j = 0; j < no_cache.numFaces(); ++j) {
        if (with_cache.face_voxels[i] != no_cache.face_voxels[j]) {
          continue;
        }

        if (faceSame(with_cache, no_cache, i, j)) {
          ++matches;
        }
      }

      EXPECT_EQ(matches, 1u);
    }
  }
}

}  // namespace hydra
