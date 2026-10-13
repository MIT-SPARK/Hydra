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
#include "hydra/frontend/delta_mesh_compression.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>

#include "hydra/active_window/active_window_output.h"
#include "hydra/utils/pgmo_mesh_traits.h"

namespace hydra {
namespace {

const auto registration =
    config::RegistrationWithConfig<MeshCompressor,
                                   DeltaMeshCompression,
                                   DeltaMeshCompression::Config>("DeltaCompression");

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

}  // namespace

using spatial_hash::IndexSet;

void declare_config(DeltaMeshCompression::Config& config) {
  using namespace config;
  name("DeltaMeshCompression::Config");
  field(config.resolution, "resolution", "m");
  check(config.resolution, GT, 0.0, "resolution");
}

DeltaMeshCompression::DeltaMeshCompression(const Config& config)
    : config(config::checkValid(config)), compression_(config.resolution) {}

auto DeltaMeshCompression::update(const ActiveWindowOutput& input,
                                  const VolumetricWindow*) -> MeshDeltaPtr {
  const auto wrapper = BlockMeshIter(input.map().getMeshLayer());
  const IndexSet archived(input.archived.begin(), input.archived.end());
  compression_.archiveBlocks(
      [&](const auto& index, const auto&) { return archived.count(index); });
  return compression_.update(wrapper, input.timestamp_ns);
}

}  // namespace hydra
