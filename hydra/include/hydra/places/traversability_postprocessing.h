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

#include <Eigen/Core>
#include <memory>

#include "hydra/active_window/active_window_output.h"
#include "hydra/places/traversability_layer.h"

namespace hydra::places {

/**
 * @brief Interface for various traversability post processing functions that can be
 * applied to the traversability layer.
 */
class TraversabilityProcessor {
 public:
  using Ptr = std::shared_ptr<TraversabilityProcessor>;
  using ConstPtr = std::shared_ptr<const TraversabilityProcessor>;

  TraversabilityProcessor() = default;
  virtual ~TraversabilityProcessor() = default;

  /**
   * @brief Apply the post processing to the traversability layer.
   * @param msg The active window output that triggered the update.
   * @param layer The traversability layer to process.
   */
  virtual void apply(const ActiveWindowOutput& msg, TraversabilityLayer& layer) = 0;
};

/**
 * @brief Processor whose changes are one-off and do not persist, i.e., applied to a
 * copy of the traversability layer.
 */
class TransientTraversabilityProcessor : public TraversabilityProcessor {
 public:
  using Ptr = std::shared_ptr<TransientTraversabilityProcessor>;
  using ConstPtr = std::shared_ptr<const TransientTraversabilityProcessor>;
};

/**
 * @brief Processor whose changes persist, i.e., applied to the stored traversability
 * layer.
 */
class PersistentTraversabilityProcessor : public TraversabilityProcessor {
 public:
  using Ptr = std::shared_ptr<PersistentTraversabilityProcessor>;
  using ConstPtr = std::shared_ptr<const PersistentTraversabilityProcessor>;
};

/**
 * @brief Utility container for several processors.
 */
template <typename ProcessorT>
struct TraversabilityProcessors {
  using Config = std::vector<config::VirtualConfig<ProcessorT>>;

  explicit TraversabilityProcessors(const Config& config) {
    for (const auto& processor_config : config) {
      auto processor = processor_config.create();
      if (processor) {
        processors_.emplace_back(std::move(processor));
      }
    }
  }

  /** @brief Apply all processors to the traversability layer.
   * @param msg The active window output that triggered the update.
   * @param layer The traversability layer to process.
   */
  void apply(const ActiveWindowOutput& msg, TraversabilityLayer& layer) const {
    for (const auto& processor : processors_) {
      processor->apply(msg, layer);
    }
  }

 private:
  std::vector<std::shared_ptr<ProcessorT>> processors_;
};

using TransientTraversabilityProcessors =
    TraversabilityProcessors<TransientTraversabilityProcessor>;
using PersistentTraversabilityProcessors =
    TraversabilityProcessors<PersistentTraversabilityProcessor>;

/**
 * @brief Dilate intraversable space in the traversability layer.
 */
class ErosionDilation : public TransientTraversabilityProcessor {
 public:
  struct Config {
    //! @brief Number of dilation steps to apply.
    size_t num_dilations = 1;
  };

  ErosionDilation(const Config& config);
  ~ErosionDilation() override = default;

  void apply(const ActiveWindowOutput& msg, TraversabilityLayer& layer) override;

  const Config config;

 protected:
  static const std::array<Index2D, 4> offsets_;
};

void declare_config(ErosionDilation::Config& config);

/**
 * @brief Guarantee that all voxels the robot footprint has ever intersected are
 * traversable. Footprint voxels are accumulated over time and re-applied on every
 * update, since the estimator resets the state of updated blocks.
 */
class RobotFootprintTraversability : public PersistentTraversabilityProcessor {
 public:
  struct Config {
    //! @brief Minimum corner of the robot footprint in body frame [m]. Only x/y are
    //! used.
    Eigen::Vector3f bbox_min = Eigen::Vector3f::Zero();
    //! @brief Maximum corner of the robot footprint in body frame [m]. Only x/y are
    //! used.
    Eigen::Vector3f bbox_max = Eigen::Vector3f::Zero();
  };

  RobotFootprintTraversability(const Config& config);
  ~RobotFootprintTraversability() override = default;

  void apply(const ActiveWindowOutput& msg, TraversabilityLayer& layer) override;

  //! @brief Add all voxels intersecting the footprint at the given pose.
  void addFootprint(const Eigen::Isometry3f& world_T_body,
                    const TraversabilityLayer& layer);

  //! @brief Mark all accumulated footprint voxels as traversable.
  void markTraversable(TraversabilityLayer& layer) const;

  const Index2DSet& footprint() const { return footprint_; }

  const Config config;

 protected:
  //! Global 2D voxel indices that were covered by the footprint at any time.
  Index2DSet footprint_;
};

void declare_config(RobotFootprintTraversability::Config& config);

}  // namespace hydra::places
