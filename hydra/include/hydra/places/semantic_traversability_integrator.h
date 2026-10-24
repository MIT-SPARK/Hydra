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

#include <map>
#include <memory>
#include <optional>
#include <utility>

#include "hydra/active_window/active_window_output.h"
#include "hydra/places/traversability_layer.h"
#include "hydra/places/traversability_postprocessing.h"
#include "hydra/reconstruction/projection_interpolators.h"

namespace hydra::places {

//! Refines traversability by projecting floor cells into the traversability image.
class ProjectiveSemanticTraversabilityIntegrator
    : public PersistentTraversabilityProcessor {
 public:
  //! How the semantic estimate combines with the geometric one.
  enum class FusionMode {
    //! Semantics may only lower traversability.
    MIN,
    //! Multiply the two probabilities.
    PRODUCT,
    //! Confidence-weighted mean of the two probabilities.
    WEIGHTED_MEAN
  };

  struct Config {
    //! Max |measured - cell range| for a cell to be visible [m]. Negative: voxel units.
    float max_surface_distance = -2.0f;

    //! Minimum |cos| between the viewing ray and the vertical, rejects grazing views.
    float min_incidence_cosine = 0.2f;

    //! Maximum observation range [m]. Non-positive values only apply the sensor range.
    float max_range = 0.0f;

    //! Observations below this weight are discarded.
    float min_measurement_weight = 1.0e-4f;

    //! Cap on accumulated per-cell weight. Lower values adapt and forget faster.
    float max_weight = 1.0e4f;

    //! Accumulated weight at which semantic confidence reaches 0.5.
    float weight_half = 10.0f;

    //! How the semantic and geometric traversability estimates combine.
    FusionMode fusion_mode = FusionMode::MIN;

    //! Which interpolation to use when sampling the input images.
    config::VirtualConfig<ProjectionInterpolator> interpolation_method{
        InterpolatorNearest::Config{}};

    //! Traversability probability per label, e.g. {3: 0.98, 12: 0.02}. Others ignored.
    std::map<uint32_t, float> label_traversability;
  } const config;

  explicit ProjectiveSemanticTraversabilityIntegrator(const Config& config);
  ~ProjectiveSemanticTraversabilityIntegrator() override = default;

  /**
   * @brief Fuse the semantic evidence in this update into the traversability layer.
   * @param msg Active window output providing sensor data and pose.
   * @param layer Geometric traversability layer to refine in place.
   */
  void apply(const ActiveWindowOutput& msg, TraversabilityLayer& layer) override;

  //! A single semantic observation of a floor cell.
  struct CellMeasurement {
    bool valid = false;
    InterpolationWeights interpolation_weights;
    float weight = 0.0f;
    int32_t label = -1;
  };

  /**
   * @brief Project a floor cell into the input images and sample its label.
   * @param data Input data for the frame.
   * @param p_S Cell center in the sensor frame.
   * @param incidence |cos| of the angle between the viewing ray and the vertical.
   * @param voxel_size Size of a floor cell in meters.
   * @return The observation, with `valid` false if the cell cannot be used.
   */
  CellMeasurement getCellMeasurement(const InputData& data,
                                     const Point& p_S,
                                     float incidence,
                                     float voxel_size) const;

  //! Fuse a single observation into the semantic fields of a voxel.
  void updateCell(const CellMeasurement& measurement, TraversabilityVoxel& voxel) const;

  /**
   * @brief Reduce a voxel's semantic fields to a (traversability, confidence) pair.
   * @return nullopt if the voxel has never been observed semantically.
   */
  std::optional<std::pair<float, float>> evaluate(
      const TraversabilityVoxel& voxel) const;

  //! Combine a voxel's semantic estimate into its geometric estimate.
  void applyToVoxel(TraversabilityVoxel& voxel) const;

  //! Resolved occlusion gate in meters. Valid after the first update.
  float maxSurfaceDistance() const { return max_surface_distance_m_; }

  //! The image the labels are read from.
  const cv::Mat& getLabelImage(const InputData& data) const;

 protected:
  void integrateFrame(const InputData& data, TraversabilityLayer& layer) const;

  void updateBlock(const InputData& data,
                   const Eigen::Isometry3f& sensor_T_world,
                   const Point& sensor_p_world,
                   float voxel_size,
                   TraversabilityBlock& block) const;

  void applyToLayer(TraversabilityLayer& layer) const;

  const std::unique_ptr<const ProjectionInterpolator> interpolator_;
  float max_surface_distance_m_ = 0.0f;
};

void declare_config(ProjectiveSemanticTraversabilityIntegrator::Config& config);

}  // namespace hydra::places
