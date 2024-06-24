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
#include <bitset>
#include <iostream>

#include "hydra/places/gvd_voxel.h"

namespace hydra::places {

struct CubeFlagExtractor {
  using IndexRotation = Eigen::Matrix<int, 3, 3>;
  /**
   * @brief Convert a 3x3 grid of voxels from row-major order to the offset iteration
   * order of spatial_hash
   */
  static std::bitset<27> fromRowMajor(const std::bitset<27>& flags_row_major);
  /**
   * @brief Convert a 3x3 grid of voxels from the offset iteration
   * order of spatial_hash to row-major order
   */
  static std::bitset<27> toRowMajor(const std::bitset<27>& flags_sh);
  /**
   * @brief Extract a 3x3 grid of voxels in the offset iteration
   * order of spatial_hash
   */
  static std::bitset<27> extract(const GvdLayer& layer,
                                 const GlobalIndex& index,
                                 uint8_t min_extra_basis = 1);

  /**
   * @brief Rotate a 3x3 grid of voxels in the offset iteration order of spatial_hash by
   * a permutation matrix
   */
  static std::bitset<27> rotate(const IndexRotation& rotation,
                                const std::bitset<27>& flags_sh);

  /**
   * @brief Offset iteration order
   */
  const static std::vector<GlobalIndex> sh_offsets;
  const static spatial_hash::IndexHashMap<size_t> sh_offset_lookup;
};

struct GvdCornerTemplate {
  using MaskArray = std::array<std::bitset<27>, 4>;

  GvdCornerTemplate();

  GvdCornerTemplate(std::bitset<27> fg_mask_row_major,
                    MaskArray unused_mask_array_row_major);

  bool matches(std::bitset<27> state) const;

  std::bitset<27> fg_mask;
  MaskArray unused_mask_array;
};

std::ostream& operator<<(std::ostream& out, const GvdCornerTemplate& corner_template);

struct CornerFinder {
  CornerFinder();

  ~CornerFinder() = default;

  bool match(std::bitset<27> values) const;

  GvdCornerTemplate negative_x_template;
  GvdCornerTemplate positive_x_template;
  GvdCornerTemplate negative_y_template;
  GvdCornerTemplate positive_y_template;
  GvdCornerTemplate negative_z_template;
  GvdCornerTemplate positive_z_template;
};

}  // namespace hydra::places
