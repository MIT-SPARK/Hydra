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

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <utility>

#include "hydra/active_window/active_window_output.h"
#include "hydra/places/traversability_layer.h"
#include "hydra/reconstruction/projection_interpolators.h"
#include "hydra/reconstruction/semantic_integrator.h"

namespace hydra::places {

/**
 * @brief Persistent per-cell semantic state for traversability estimation.
 * @note This deliberately lives in a layer owned by the integrator rather than in
 * TraversabilityVoxel, because the traversability estimators call Block2D::reset() on
 * every updated block each frame, which would destroy any accumulated belief.
 */
struct SemanticTraversabilityCell {
  //! Fused label belief. Reusing SemanticVoxel means every existing SemanticIntegrator
  //! (MLE, binary, first-k, single-label) applies without modification.
  SemanticVoxel semantics;

  //! Total accumulated observation weight. Drives the reported confidence.
  float weight = 0.0f;
};

using SemanticTraversabilityBlock = Block2D<SemanticTraversabilityCell>;
using SemanticTraversabilityLayer = Layer2D<SemanticTraversabilityCell>;

/**
 * @brief Maps a semantic label to the probability that the labelled surface can be
 * traversed.
 * @note This is intentionally separate from the label space: whether stairs or grass are
 * traversable is a property of the robot, not of the segmentation model.
 */
class TraversabilityPrior {
 public:
  using Ptr = std::unique_ptr<const TraversabilityPrior>;

  TraversabilityPrior() = default;
  virtual ~TraversabilityPrior() = default;

  /**
   * @brief Probability that a surface with the given label is traversable.
   * @param label Semantic label to look up.
   * @return The probability in (0, 1), or nullopt if the label carries no information
   * (in which case no semantic update is performed for the observation).
   */
  virtual std::optional<float> probability(uint32_t label) const = 0;
};

/**
 * @brief Traversability prior from an explicit per-label probability table.
 */
class LabelTablePrior : public TraversabilityPrior {
 public:
  struct Config {
    //! @brief Per-label traversability probabilities, e.g. {3: 0.98, 12: 0.02}.
    std::map<uint32_t, float> label_traversability;

    //! @brief Probability assigned to labels absent from the table. Only used when
    //! `ignore_unknown_labels` is false. 0.5 means "no information".
    float default_traversability = 0.5f;

    //! @brief If true, labels absent from the table are skipped entirely rather than
    //! fused at `default_traversability`.
    bool ignore_unknown_labels = true;
  } const config;

  explicit LabelTablePrior(const Config& config);
  ~LabelTablePrior() override = default;

  std::optional<float> probability(uint32_t label) const override;
};

void declare_config(LabelTablePrior::Config& config);

/**
 * @brief Traversability prior for label spaces where only the traversable /
 * untraversable split is known.
 */
class LabelSetPrior : public TraversabilityPrior {
 public:
  struct Config {
    //! @brief Labels considered traversable.
    std::set<uint32_t> traversable_labels;

    //! @brief Labels considered untraversable.
    std::set<uint32_t> untraversable_labels;

    //! @brief Probability assigned to a label in either set. Labels in neither set carry
    //! no information.
    float confidence = 0.9f;
  } const config;

  explicit LabelSetPrior(const Config& config);
  ~LabelSetPrior() override = default;

  std::optional<float> probability(uint32_t label) const override;
};

void declare_config(LabelSetPrior::Config& config);

/**
 * @brief Interface for refining a geometric traversability estimate with semantics.
 */
class SemanticTraversabilityIntegrator {
 public:
  using Ptr = std::unique_ptr<SemanticTraversabilityIntegrator>;

  //! Callback used to recompute TraversabilityState after fusion. Supplied by the caller
  //! so that the integrator does not need to know the estimator's thresholds.
  using Classifier = std::function<void(TraversabilityVoxel&)>;

  SemanticTraversabilityIntegrator() = default;
  virtual ~SemanticTraversabilityIntegrator() = default;

  /**
   * @brief Fuse the semantic evidence in this frame into the traversability layer.
   * @param msg Active window output providing sensor data and pose.
   * @param layer Geometric traversability layer to refine in place.
   * @param reclassify Callback to recompute the discrete state of a modified voxel.
   */
  virtual void updateTraversability(const ActiveWindowOutput& msg,
                                    TraversabilityLayer& layer,
                                    const Classifier& reclassify) = 0;

  //! @brief Accumulated semantic state, for visualization and debugging. May be null.
  virtual const SemanticTraversabilityLayer* getSemanticLayer() const { return nullptr; }
};

/**
 * @brief Refines traversability by projecting each floor cell into the semantic label
 * image and recursively fusing the observed labels.
 *
 * Plays the role ProjectiveIntegrator plays for the TSDF, but over the 2D floor cells
 * produced by a TraversabilityEstimator. Each cell's stored surface height gives it a
 * full 3D position, which is projected into the label image; the sampled label is fused
 * through a SemanticIntegrator and reduced to a traversability probability through a
 * TraversabilityPrior.
 */
class ProjectiveSemanticTraversabilityIntegrator
    : public SemanticTraversabilityIntegrator {
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

    //! @brief Minimum |cos| of the angle between the viewing ray and the vertical for an
    //! observation to be used. Rejects grazing views, where a single pixel covers many
    //! floor cells.
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

    //! @brief Number of threads used for integration (parallelized by block).
    int num_threads = 1;

    //! @brief Which interpolation to use when sampling the input images.
    config::VirtualConfig<ProjectionInterpolator> interpolation_method{
        InterpolatorNearest::Config{}};

    //! @brief How repeated label observations of the same cell are accumulated.
    config::VirtualConfig<SemanticIntegrator> semantic_integrator{
        MLESemanticIntegrator::Config{}};

    //! @brief How a fused label is turned into a traversability probability.
    config::VirtualConfig<TraversabilityPrior> prior;
  } const config;

  explicit ProjectiveSemanticTraversabilityIntegrator(const Config& config);
  ~ProjectiveSemanticTraversabilityIntegrator() override = default;

  void updateTraversability(const ActiveWindowOutput& msg,
                            TraversabilityLayer& layer,
                            const Classifier& reclassify) override;

  const SemanticTraversabilityLayer* getSemanticLayer() const override {
    return state_.get();
  }

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
   * @param data Input data for the current frame.
   * @param p_S Cell center in the sensor frame.
   * @param incidence |cos| of the angle between the viewing ray and the vertical.
   * @return The observation, with `valid` false if the cell cannot be used.
   */
  CellMeasurement getCellMeasurement(const InputData& data,
                                     const Point& p_S,
                                     float incidence) const;

  /**
   * @brief Fuse a single observation into the persistent state of a cell.
   */
  void updateCell(const CellMeasurement& measurement,
                  SemanticTraversabilityCell& cell) const;

  /**
   * @brief Reduce accumulated state to a (traversability, confidence) pair.
   * @return nullopt if the cell has never been observed or carries no information.
   */
  std::optional<std::pair<float, float>> evaluate(
      const SemanticTraversabilityCell& cell) const;

  /**
   * @brief Combine a cell's semantic estimate into the corresponding geometric voxel.
   */
  void applyToVoxel(const SemanticTraversabilityCell& cell,
                    TraversabilityVoxel& voxel) const;

  //! @brief Resolved occlusion gate in meters. Valid after the first update.
  float maxSurfaceDistance() const { return max_surface_distance_m_; }

 protected:
  //! @brief Allocate the state layer to match the geometry layer's grid.
  void initialize(const TraversabilityLayer& layer);

  //! @brief Drop state for blocks no longer present in the geometry layer.
  void pruneArchived(const TraversabilityLayer& layer);

  //! @brief Integrate one block of floor cells.
  void updateBlock(const BlockIndex& block_index,
                   const InputData& data,
                   const Eigen::Isometry3f& sensor_T_world,
                   const Point& sensor_p_world,
                   const TraversabilityLayer& layer);

  //! @brief Write accumulated semantics into the geometry layer and reclassify.
  void applyToLayer(TraversabilityLayer& layer, const Classifier& reclassify) const;

  std::unique_ptr<SemanticTraversabilityLayer> state_;
  const std::unique_ptr<const ProjectionInterpolator> interpolator_;
  const std::unique_ptr<const SemanticIntegrator> semantic_integrator_;
  const TraversabilityPrior::Ptr prior_;
  float max_surface_distance_m_ = 0.0f;
  bool warned_missing_height_ = false;
};

void declare_config(ProjectiveSemanticTraversabilityIntegrator::Config& config);

}  // namespace hydra::places
