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

#include <config_utilities/virtual_config.h>

#include <cstdint>
#include <memory>

#include "hydra/places/traversability_postprocessing.h"
#include "hydra/reconstruction/projection_interpolators.h"

namespace hydra::places {

/**
 * @brief Fuse a per-pixel semantic traversability image into the 2.5D traversability
 * layer.
 *
 * Every traversability voxel carries a surface height, so it has a well-defined 3D
 * position (x, y, height) that can be projected into the camera. This processor
 * projects each such cell, checks it is actually visible by comparing its range
 * against the range image, interpolates the label image at that pixel, and counts the
 * observation. The per-cell traversable/intraversable counts are accumulated across
 * frames in a shadow layer owned by this processor, and stamped onto the voxels as
 * `semantic_traversability` and `semantic_confidence`.
 *
 * @note The geometric `traversability`, `confidence` and `state` fields are left
 * untouched, so this can be enabled alongside any estimator without changing existing
 * behavior. No fusion rule is applied yet.
 *
 * @note This is deliberately NOT a TSDF integrator. Because the layer is 2.5D there is
 * no signed distance to accumulate, hence no truncation band, no weight dropoff and no
 * depth-dependent measurement weight. What remains from projective integration is the
 * projection itself plus a visibility test.
 */
class TraversabilityProjectiveIntegrator : public TraversabilityProcessor {
 public:
  struct Config {
    //! @brief Label treated as traversable evidence.
    int traversable_label = 1;

    //! @brief Label treated as intraversable evidence. Any other label (e.g. an
    //! "unknown" -1) is ignored rather than counted as evidence either way.
    int intraversable_label = 0;

    //! @brief Number of observations at which semantic_confidence saturates to 1.
    int confidence_saturation_count = 5;

    //! @brief Maximum |voxel_range - measured_range| for a cell to count as visible in
    //! meters. Negative values are multiples of the voxel size. This is a depth-buffer
    //! agreement test, not a signed distance: a positive error means the cell sits
    //! behind the observed surface (occluded), a negative one that the depth image says
    //! nothing is there (stale height).
    float max_range_error = -1.0f;

    //! @brief Which interpolation to use in the image projection [nearest, bilinear,
    //! adaptive].
    config::VirtualConfig<ProjectionInterpolator> interpolation_method{
        InterpolatorAdaptive::Config{}};

    //! @brief If true, also write semantic_traversability into the voxel's debug_value
    //! scaled into the visualizer's rainbow id range, so the existing "debug" marker
    //! shows the semantic channel without any visualizer changes.
    bool write_debug_value = false;
  } const config;

  explicit TraversabilityProjectiveIntegrator(const Config& config);
  ~TraversabilityProjectiveIntegrator() override = default;

  void apply(TraversabilityLayer& layer, const ActiveWindowOutput& msg) override;

 protected:
  //! @brief Per-cell label counts. These live in the processor rather than in the
  //! layer because the layer handed to apply() is a per-tick copy whose blocks the
  //! estimator resets on every update.
  struct LabelCounts {
    uint32_t traversable = 0;
    uint32_t intraversable = 0;

    uint32_t total() const { return traversable + intraversable; }
  };
  using CountLayer = Layer2D<LabelCounts>;

  //! Allocated lazily on the first apply() from the incoming layer's grid, so the two
  //! layers are index-identical.
  std::unique_ptr<CountLayer> counts_;
  std::unique_ptr<ProjectionInterpolator> interpolator_;
};

void declare_config(TraversabilityProjectiveIntegrator::Config& config);

}  // namespace hydra::places
