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

#include <memory>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "hydra/places/traversability_postprocessing.h"
#include "hydra/reconstruction/projection_interpolators.h"

namespace hydra {
struct InputData;
}  // namespace hydra

namespace hydra::places {

/**
 * @brief Fuse per-pixel semantic traversability images into the 2.5D traversability
 * layer.
 *
 * Every traversability voxel with an observed surface carries a height, so it has a
 * well-defined 3D position (x, y, height) that can be projected into the camera. This
 * integrator projects each such cell, checks it is actually visible by comparing its
 * range against the range image, interpolates the label image at that pixel, and counts
 * the observation in the voxel's `semantic` field. The labels are read from either the
 * dedicated traversability image (e.g., from a traversability segmentation network
 * such as GA-Nav) or the semantic label image.
 *
 * @note The semantic counts live on the voxels and persist across updates: the
 * extractor carries the semantic fields of the postprocessed layer back to the
 * estimator's layer, and the estimators reset only the geometric fields (see
 * resetGeometry()).
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
  enum class InputImage {
    TRAVERSABILITY,  //!< InputData::traversability_image
    LABEL,           //!< InputData::label_image
  };

  struct Config {
    //! @brief Which image of the input data to read the labels from.
    InputImage input_image = InputImage::TRAVERSABILITY;

    //! @brief Labels treated as traversable evidence.
    std::vector<int> traversable_labels{1};

    //! @brief Labels treated as intraversable evidence. Any other label (e.g. an
    //! "unknown" -1) is ignored rather than counted as evidence either way.
    std::vector<int> intraversable_labels{0};

    //! @brief Number of observations at which semantic.confidence saturates to 1.
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

    //! @brief If true, also write the semantic traversability into the voxel's
    //! debug_value, scaled by debug_value_scale, so the existing "debug" marker shows
    //! the semantic channel without any visualizer changes.
    bool write_debug_value = false;

    //! @brief Scale applied to the semantic traversability in [0, 1] when writing the
    //! debug value. The default spans the visualizer's rainbow id range.
    float debug_value_scale = 9.0f;
  } const config;

  explicit TraversabilityProjectiveIntegrator(const Config& config);
  ~TraversabilityProjectiveIntegrator() override = default;

  void apply(TraversabilityLayer& layer, const ActiveWindowOutput& msg) override;

 protected:
  std::unique_ptr<ProjectionInterpolator> interpolator_;

  //! Get the configured label image of the input data.
  const cv::Mat& labelImage(const InputData& data) const;

  //! Fuse one frame of sensor data into the layer.
  void integrateFrame(TraversabilityLayer& layer, const InputData& data) const;
};

void declare_config(TraversabilityProjectiveIntegrator::Config& config);

}  // namespace hydra::places
