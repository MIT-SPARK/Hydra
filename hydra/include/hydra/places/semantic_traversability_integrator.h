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

/**
 * @brief Refines traversability by projecting each floor cell into the semantic label
 * image and recursively fusing the observed labels.
 *
 * Plays the role ProjectiveIntegrator plays for the TSDF, but over the 2D floor cells
 * produced by a TraversabilityEstimator. Each cell's stored surface height gives it a
 * full 3D position, which is projected into the label image; the sampled label is
 * mapped to a traversability probability through `label_traversability` and fused into
 * the semantic fields of the TraversabilityVoxel.
 */
class ProjectiveSemanticTraversabilityIntegrator
    : public PersistentTraversabilityProcessor {
 public:
  //! How the semantic estimate combines with the geometric one.
  enum class FusionMode {
    //! Semantics may only lower traversability. Geometry stays authoritative.
    MIN,
    //! Multiply the two probabilities.
    PRODUCT,
    //! Confidence-weighted mean. Allows semantics to raise traversability, e.g. to
    //! rescue a rough but drivable surface such as gravel or lawn.
    WEIGHTED_MEAN
  };

  struct Config {
    //! @brief Maximum |measured range - cell range| for a cell to count as visible in
    //! the current frame. Negative values are multiples of the voxel size. This is the
    //! occlusion gate: without it, labels of objects in front of the floor (tables,
    //! people) are painted onto the floor beneath them.
    float max_surface_distance = -2.0f;

    //! @brief Minimum |cos| of the angle between the viewing ray and the vertical for
    //! an observation to be used. Rejects grazing views, where a single pixel covers
    //! many floor cells.
    float min_incidence_cosine = 0.2f;

    //! @brief Observations below this weight are discarded.
    float min_measurement_weight = 1.0e-4f;

    //! @brief Saturating cap on accumulated per-cell weight. Lower values adapt to
    //! change faster but forget faster.
    float max_weight = 1.0e4f;

    //! @brief Accumulated weight at which semantic confidence reaches 0.5.
    float weight_half = 10.0f;

    //! @brief How the semantic and geometric traversability estimates combine.
    FusionMode fusion_mode = FusionMode::MIN;

    //! @brief Which interpolation to use when sampling the input images.
    config::VirtualConfig<ProjectionInterpolator> interpolation_method{
        InterpolatorNearest::Config{}};

    //! @brief Probability in [0, 1] that a surface with each label is traversable, e.g.
    //! {3: 0.98, 12: 0.02}. Labels not listed carry no information and are ignored.
    //! This is kept separate from the label space: whether stairs or grass are
    //! traversable is a property of the robot, not of the segmentation model.
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

  /**
   * @brief A single semantic observation of a floor cell.
   * @note Analogous to ProjectiveIntegrator::VoxelMeasurement.
   */
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

  /**
   * @brief Fuse a single observation into the semantic fields of a voxel.
   */
  void updateCell(const CellMeasurement& measurement, TraversabilityVoxel& voxel) const;

  /**
   * @brief Reduce a voxel's semantic fields to a (traversability, confidence) pair.
   * @return nullopt if the voxel has never been observed semantically.
   */
  std::optional<std::pair<float, float>> evaluate(
      const TraversabilityVoxel& voxel) const;

  /**
   * @brief Combine a voxel's semantic estimate into its geometric estimate.
   */
  void applyToVoxel(TraversabilityVoxel& voxel) const;

  //! @brief Resolved occlusion gate in meters. Valid after the first update.
  float maxSurfaceDistance() const { return max_surface_distance_m_; }

 protected:
  //! @brief Integrate the labels of a single frame into the layer.
  void integrateFrame(const InputData& data, TraversabilityLayer& layer) const;

  //! @brief Integrate one block of floor cells.
  void updateBlock(const InputData& data,
                   const Eigen::Isometry3f& sensor_T_world,
                   const Point& sensor_p_world,
                   float voxel_size,
                   TraversabilityBlock& block) const;

  /**
   * @brief Fuse accumulated semantics into the geometric estimate. The estimator
   * classifies the result once all persistent processing has run.
   * @note Only blocks marked as updated are touched: the estimators only recompute
   * those blocks, so fusing into the others would apply the semantics on top of an
   * estimate that already contains them.
   */
  void applyToLayer(TraversabilityLayer& layer) const;

  const std::unique_ptr<const ProjectionInterpolator> interpolator_;
  float max_surface_distance_m_ = 0.0f;
};

void declare_config(ProjectiveSemanticTraversabilityIntegrator::Config& config);

}  // namespace hydra::places
