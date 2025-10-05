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

#include <config_utilities/factory.h>
#include <kimera_pgmo/mesh_delta.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <memory>

#include "hydra/active_window/active_window_output.h"
#include "hydra/places/traversability_layer.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

class TraversabilityEstimator {
 public:
  using Ptr = std::shared_ptr<TraversabilityEstimator>;
  using ConstPtr = std::shared_ptr<const TraversabilityEstimator>;

  TraversabilityEstimator() = default;
  virtual ~TraversabilityEstimator() = default;

  /**
   * @brief Update the traversability based on the active window output and mesh delta.
   * @param msg The active window output containing the latest sensor data and
   * volumetric map updates.
   * @param mesh_delta The mesh delta representing changes in the mesh.
   * @param graph The dynamic scene graph to update with traversability information.
   */
  virtual void updateTraversability(const ActiveWindowOutput& msg,
                                    const kimera_pgmo::MeshDelta& mesh_delta,
                                    const spark_dsg::DynamicSceneGraph& graph) = 0;

  virtual const TraversabilityLayer& getTraversabilityLayer() const {
    return *traversability_layer_;
  }

 protected:
  std::unique_ptr<TraversabilityLayer> traversability_layer_;
};

/**
 * @brief Simple traversability estimator which checks a specified volume in the TSDF
 * map for obstacles.
 * @note Assumptions: Only works for flat ground, and does not consider cliffs as
 * obstacles.
 */
class HeightTraversabilityEstimator : public TraversabilityEstimator {
 public:
  struct Config {
    //! @brief The height above the robot body to consider for traversability in meters.
    float height_above = 0.5f;

    //! @brief The height below the robot body to consider for traversability in meters.
    float height_below = 0.5f;

    //! @brief Minimum confidence for a voxel to be considered observed.
    float min_confidence = 1.0f;

    //! @brief Minimum traversability for a voxel to be considered traversable.
    float min_traversability = 1.0f;

    //! @brief If true, mark voxels as intraversable if they do not meet the
    //! min_traversability threshold, even if the confidence is below min_confidence. If
    //! false, mark these voxels as unknown instead.
    bool pessimistic = true;
  };

  HeightTraversabilityEstimator(const Config& config);
  ~HeightTraversabilityEstimator() override = default;

  void updateTraversability(const ActiveWindowOutput& msg,
                            const kimera_pgmo::MeshDelta& /* mesh_delta */,
                            const spark_dsg::DynamicSceneGraph& /* graph */) override;

  const Config config;

 protected:
  TsdfLayer::Ptr tsdf_layer_;

  // Processing steps.
  void updateTsdf(const ActiveWindowOutput& msg);
  void computeTraversability(const ActiveWindowOutput& msg);
  void classifyTraversabilityVoxel(TraversabilityVoxel& voxel) const;

  // Helper functions.
  BlockIndexSet get2DBlockIndices(const BlockIndices& blocks) const;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<TraversabilityEstimator,
                                     HeightTraversabilityEstimator,
                                     Config>("HeightTraversabilityEstimator");
};

void declare_config(HeightTraversabilityEstimator::Config& config);

}  // namespace hydra::places
