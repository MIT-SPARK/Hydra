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
#include "hydra/places/update_statistics.h"

namespace hydra::places {

void UpdateStatistics::clear() {
  number_surface_flipped = 0;
  number_lowered_voxels = 0;
  number_raised_voxels = 0;
  number_new_voxels = 0;
  number_raise_updates = 0;
  number_voronoi_found = 0;
  number_lower_skipped = 0;
  number_lower_updated = 0;
  number_fixed_no_parent = 0;
  number_force_lowered = 0;
}

std::ostream& operator<<(std::ostream& out, const UpdateStatistics& stats) {
  out << "  - Invalid surface flags: " << stats.number_surface_flipped << std::endl;
  out << "  - Voxel changes: ";
  out << stats.number_lowered_voxels << " lowered, ";
  out << stats.number_raised_voxels << " raised, ";
  out << stats.number_new_voxels << " new" << std::endl;
  out << "  - New Voronoi Cells: " << stats.number_voronoi_found << std::endl;
  out << "  - Fixed without parents (lower): " << stats.number_fixed_no_parent
      << std::endl;
  out << "  - Skipped (lower): " << stats.number_lower_skipped << std::endl;
  out << "  - Updated (lower): " << stats.number_lower_updated << std::endl;
  out << "  - Forced (lower): " << stats.number_force_lowered << std::endl;
  return out;
}

}  // namespace hydra::places
