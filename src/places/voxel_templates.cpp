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
#include "hydra/places/voxel_templates.h"

#include <spatial_hash/neighbor_utils.h>

#include "hydra/common/common.h"

namespace hydra::places {

namespace {

inline bool isValidPoint(const GvdVoxel* voxel, uint8_t min_extra_basis) {
  if (!voxel) {
    return false;
  }

  return voxel->num_extra_basis >= min_extra_basis;
}

inline std::vector<GlobalIndex> get26ConnectedOffsets() {
  const spatial_hash::NeighborSearch search(26);
  return search.neighborIndices(GlobalIndex::Zero().eval(), true);
}

inline spatial_hash::IndexHashMap<size_t> get26ConnectedLookup() {
  const spatial_hash::NeighborSearch search(26);
  const auto indices = search.neighborIndices(GlobalIndex::Zero().eval(), true);

  spatial_hash::IndexHashMap<size_t> lookup;
  for (size_t i = 0; i < indices.size(); ++i) {
    lookup[indices[i]] = i;
  }

  return lookup;
}

}  // namespace

const std::vector<GlobalIndex> CubeFlagExtractor::sh_offsets = get26ConnectedOffsets();
const spatial_hash::IndexHashMap<size_t> CubeFlagExtractor::sh_offset_lookup =
    get26ConnectedLookup();

std::bitset<27> CubeFlagExtractor::fromRowMajor(const std::bitset<27>& flags_rm) {
  std::bitset<27> flags_sh{0};
  for (size_t i = 1; i < sh_offsets.size(); ++i) {
    // x -> y -> z ordering (plus shift from [-1, 1] to [0, 2] to reflect lower corner
    // origin)
    size_t row_major_index = 1 * static_cast<size_t>(sh_offsets[i].x() + 1) +
                             3 * static_cast<size_t>(sh_offsets[i].y() + 1) +
                             9 * static_cast<size_t>(sh_offsets[i].z() + 1);

    flags_sh.set(i, flags_rm[26 - row_major_index]);
  }

  flags_sh.set(0, flags_rm[13]);
  return flags_sh;
}

std::bitset<27> CubeFlagExtractor::toRowMajor(const std::bitset<27>& flags_sh) {
  std::bitset<27> flags_rm{0};
  for (size_t i = 1; i < sh_offsets.size(); ++i) {
    // x -> y -> z ordering (plus shift from [-1, 1] to [0, 2] to reflect lower corner
    // origin)
    size_t row_major_index = 1 * static_cast<size_t>(sh_offsets[i].x() + 1) +
                             3 * static_cast<size_t>(sh_offsets[i].y() + 1) +
                             9 * static_cast<size_t>(sh_offsets[i].z() + 1);

    flags_rm.set(26 - row_major_index, flags_sh[i]);
  }

  flags_rm.set(13, flags_sh[0]);
  return flags_rm;
}

std::bitset<27> CubeFlagExtractor::extract(const GvdLayer& layer,
                                           const GlobalIndex& index,
                                           uint8_t min_extra_basis) {
  std::bitset<27> neighbor_values;
  for (size_t n = 0; n < sh_offsets.size(); ++n) {
    const auto* voxel = layer.getVoxelPtr((sh_offsets[n] + index).eval());
    neighbor_values.set(n, isValidPoint(voxel, min_extra_basis));
  }

  return neighbor_values;
}

std::bitset<27> CubeFlagExtractor::rotate(const IndexRotation& rotation,
                                          const std::bitset<27>& flags_sh) {
  std::bitset<27> rotated{0};
  rotated.set(0, flags_sh[0]);  // center voxel will always remain the same
  for (size_t i = 0; i < sh_offsets.size(); ++i) {
    const GlobalIndex rotated_index = rotation * sh_offsets[i];
    size_t rotated_idx = sh_offset_lookup.at(rotated_index);
    rotated[rotated_idx] = flags_sh[i];
  }

  return rotated;
}

// by default, a template only passes if all points in the 3x3 cube are voronoi
GvdCornerTemplate::GvdCornerTemplate() : fg_mask(0x7FFF'FFFF) {
  unused_mask_array = MaskArray{0, 0, 0, 0};
}

GvdCornerTemplate::GvdCornerTemplate(std::bitset<27> fg_mask_rm,
                                     GvdCornerTemplate::MaskArray unused_mask_rm)
    : fg_mask(CubeFlagExtractor::fromRowMajor(fg_mask_rm)) {
  for (size_t i = 0; i < 4; ++i) {
    unused_mask_array[i] = CubeFlagExtractor::fromRowMajor(unused_mask_rm[i]);
  }
}

bool GvdCornerTemplate::matches(std::bitset<27> state) const {
  // this computes a bitset where every voxel that matches the status flag is 1
  // this is equivalent to the bitwise and (to get gvd voxels) and the bitwise not-or
  // (to get non-gvd voxels)
  std::bitset<27> intermediate = (state & fg_mask) | ~(state | fg_mask);
  // flags we don't care about get bitwise-or'd and then each result is checked
  return (intermediate | unused_mask_array[0]).all() ||
         (intermediate | unused_mask_array[1]).all() ||
         (intermediate | unused_mask_array[2]).all() ||
         (intermediate | unused_mask_array[3]).all();
}

std::ostream& operator<<(std::ostream& out, const GvdCornerTemplate& corner_template) {
  out << "Foreground / Background mask: " << std::endl;
  out << "  - " << corner_template.fg_mask << std::endl;
  out << "Unused mask: " << std::endl;
  out << "  -   0 degrees: " << corner_template.unused_mask_array[0] << std::endl;
  out << "  -  90 degrees: " << corner_template.unused_mask_array[1] << std::endl;
  out << "  - 180 degrees: " << corner_template.unused_mask_array[2] << std::endl;
  out << "  - 270 degrees: " << corner_template.unused_mask_array[3] << std::endl;
  return out;
}

CornerFinder::CornerFinder() {
  // format for each template is {fg_mask, {0, 90, 180, 270}} where 0, 90, 180, and 270
  // represent rotations of the unused_mask around the current axis of the corresponding
  // amount of degrees (following the right-hand rule around the axis described by the
  // fg_mask template).
  // for the actual bitset values: we follow a row-major layout (matrix "shape" z, y, x)
  // where 9 * z + 3 * y + x gives the index of the bit that describes that voxel. This
  // means that you can read each mask as a group of 9 "vectors" in three groups: the
  // slices z=0, z=1, and z=2 where each vector points along the x-axis. The first
  // "vector" starts at the lower left corner of the 3x3x3 grid and ends at the lower
  // right corner.
  negative_x_template = {0b000'000'000'000'110'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'110'110'000'000'110'000'000'000,
                          0b110'110'000'110'000'000'000'000'000,
                          0b000'000'000'110'000'000'110'110'000,
                          0b000'000'000'000'000'110'000'110'110}};

  positive_x_template = {0b000'000'000'000'011'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'011'011'000'000'011'000'000'000,
                          0b000'000'000'000'000'011'000'011'011,
                          0b000'000'000'011'000'000'011'011'000,
                          0b011'011'000'011'000'000'000'000'000}};

  negative_y_template = {0b000'000'000'010'010'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b110'110'000'100'100'000'000'000'000,
                          0b011'011'000'001'001'000'000'000'000,
                          0b000'000'000'001'001'000'011'011'000,
                          0b000'000'000'100'100'000'110'110'000}};

  positive_y_template = {0b000'000'000'000'010'010'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'011'011'000'001'001'000'000'000,
                          0b000'110'110'000'100'100'000'000'000,
                          0b000'000'000'000'100'100'000'110'110,
                          0b000'000'000'000'001'001'000'011'011}};

  negative_z_template = {0b000'010'000'000'010'000'000'000'000,
                         ///////////  unused masks ////////////
                         {0b000'001'011'000'001'011'000'000'000,
                          0b011'001'000'011'001'000'000'000'000,
                          0b110'100'000'110'100'000'000'000'000,
                          0b000'100'110'000'100'110'000'000'000}};

  positive_z_template = {0b000'000'000'000'010'000'000'010'000,
                         ///////////  unused masks ////////////
                         {0b000'000'000'000'100'110'000'100'110,
                          0b000'000'000'110'100'000'110'100'000,
                          0b000'000'000'011'001'000'011'001'000,
                          0b000'000'000'000'001'011'000'001'011}};
}

bool CornerFinder::match(std::bitset<27> values) const {
  return negative_x_template.matches(values) || positive_x_template.matches(values) ||
         negative_y_template.matches(values) || positive_y_template.matches(values) ||
         negative_z_template.matches(values) || positive_z_template.matches(values);
}

}  // namespace hydra::places
