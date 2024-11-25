// The contents of this file are originally from Panoptic-Mapping,
// under the following license:
//
// BSD 3-Clause License
// Copyright (c) 2021, ETHZ ASL
// All rights reserved.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// See https://github.com/ethz-asl/panoptic_mapping for original code and paper
//
// Modifications (including work done by Lukas Schmid for Khronos) fall under the same
// license as Hydra and are subject to the following copyright and disclaimer:
//
// Copyright 2022 Massachusetts Institute of Technology.
// All Rights Reserved
//
// Research was sponsored by the United States Air Force Research Laboratory and
// the United States Air Force Artificial Intelligence Accelerator and was
// accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
// and conclusions contained in this document are those of the authors and should
// not be interpreted as representing the official policies, either expressed or
// implied, of the United States Air Force or the U.S. Government. The U.S.
// Government is authorized to reproduce and distribute reprints for Government
// purposes notwithstanding any copyright notation herein.
#pragma once
#include <memory>
#include <string>
#include <thread>

#include "hydra/common/global_info.h"
#include "hydra/input/input_packet.h"
#include "hydra/reconstruction/projection_interpolators.h"
#include "hydra/reconstruction/semantic_integrator.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

/**
 * Originally based on the following paper:
 * Lukas Schmid, Jeffrey Delmerico, Johannes Schönberger, Juan Nieto, Marc
 * Pollefeys, Roland Siegwart, and Cesar Cadena. "Panoptic Multi-TSDFs: a Flexible
 * Representation for Online Multi-resolution Volumetric Mapping and Long-term Dynamic
 * Scene Consistency" in IEEE International Conference on Robotics and Automation
 * (ICRA), pp. 8018-8024, 2022.
 */
class ProjectiveIntegrator {
 public:
  struct Config {
    //! Verbosity for the projective integrator
    int verbosity = GlobalInfo::instance().getConfig().default_verbosity;
    //! If nonzero, integrates negative voxels outside of the truncation distance by the
    //! specified threshold. Negative values are multiples of the voxel size
    float extra_integration_distance = 0.0f;
    //! If true, drop off the weight behind the surface crossing
    bool use_weight_dropoff = true;
    //! Distance in meters where the weight drop-off reaches zero. Negative
    //! values are multiples of the voxel size
    float weight_dropoff_epsilon = -1.0f;
    //! If true, use unitary (w=1) weights to update the TSDF. Otherwise use
    //! weights as a function of the squared depth to approximate typical RGBD
    //! sensor confidence
    bool use_constant_weight = false;
    //! Force measurements to carry at least a minimum weight for integration
    //! (corresponds to minimum weight required for a voxel to be observed in the mesh
    //! integrator)
    float min_measurement_weight = 1.0e-4f;
    //! Maximum weight used for TSDF updates. High max weight keeps information
    //! longer in memory, low max weight favors rapid updates
    float max_weight = 1.0e5f;
    //! Number of threads used to perform integration (parallelized by block)
    int num_threads = GlobalInfo::instance().getConfig().default_num_threads;
    //! Which interpolation to use in the image projection [nearest, bilinear,
    //! adaptive]
    config::VirtualConfig<ProjectionInterpolator> interpolation_method{
        InterpolatorAdaptive::Config{}};
    //! Semantic integrator configuration (optional)
    config::VirtualConfig<SemanticIntegrator> semantic_integrator;
  } const config;

  struct VoxelMeasurement {
    bool valid = false;
    InterpolationWeights interpolation_weights;
    float sdf = 0.0f;
    float weight = 0.0f;
    int32_t label = -1;
  };

  explicit ProjectiveIntegrator(const ProjectiveIntegrator::Config& config);

  virtual ~ProjectiveIntegrator() = default;

  /**
   * @brief Update all specified blocks in the background map with the given data in
   * parallel.
   * @param data Input data to use for the update.
   * @param map Map to update.
   * @param allocate_blocks Allocate blocks to update before integrating
   */
  void updateMap(const InputData& data,
                 VolumetricMap& map,
                 bool allocate_blocks = true) const;

  /**
   * @brief Update all specified blocks in the map with the given data in parallel.
   * @param block_indices List of block indices to update.
   * @param data Input data to use for the update.
   * @param map Map to update.
   */
  void updateBlocks(const BlockIndices& block_indices,
                    const InputData& data,
                    VolumetricMap& map) const;

  /**
   * @brief Update the specified block in the map with the given data single-threaded.
   * @param block_index Index of block to update.
   * @param data Input data to use for the update.
   * @param map Map to update.
   */
  void updateBlock(const BlockIndex& block_index,
                   const InputData& data,
                   VolumetricMap& map) const;

  /**
   * @brief Compute the data needed to update a TSDF voxel.
   * @param map_config Configuration containing truncation distance, voxel size,
   * and other map parameters.
   * @param p_C Center point of the voxel in camera (C) frame.
   * @param data Input data to use for the update.
   * @return The measurement weight that can be applied to a voxel.
   */
  VoxelMeasurement getVoxelMeasurement(const VolumetricMap::Config& map_config,
                                       const InputData& data,
                                       const Point& p_C) const;

  /**
   * @brief Update a voxel with the given measurement.
   * @param data Input data to use for the update.
   * @param measurement Measurement to use for the update.
   * @param voxels Voxel to update.
   */
  void updateVoxel(const VolumetricMap::Config& map_config,
                   const InputData& data,
                   const VoxelMeasurement& measurement,
                   VoxelTuple& voxels) const;

  /**
   * @brief Check whether the point is valid to be updated and setup the interpolation
   * weights.
   * @param p_C Center point of the voxel in camera (C) frame.
   * @param data Input data to use for the update.
   * @param weights Where to write the resulting interpolation weights to.
   * @returns True if the point is valid, false otherwise.
   */
  bool interpolatePoint(const InputData& data,
                        const Point& p_C,
                        InterpolationWeights& weights) const;

  /**
   * @brief Compute the signed distance value for the given point.
   */
  float computeSDF(const VolumetricMap::Config& map_config,
                   const InputData& data,
                   const InterpolationWeights& weights,
                   const float distance_to_voxel) const;

  /**
   * @brief Compute the TSDF update weight for the given point.
   */
  float computeWeight(const VolumetricMap::Config& map_config,
                      const Sensor& sensor,
                      const Point& p_C,
                      const float sdf) const;

  // TODO(lschmid): Find a good way to clean this up and integrate this more nicely.
  // Just adding hooks here for now for Khronos updates.
  /**
   * @brief Compute the semantic label of the given measurement.
   * @returns True if the measurement is valid for integration, false otherwise.
   */
  virtual bool computeLabel(const VolumetricMap::Config& map_config,
                            const InputData& data,
                            VoxelMeasurement& measurement) const;

 protected:
  const std::unique_ptr<const ProjectionInterpolator> interpolator_;
  const std::unique_ptr<const SemanticIntegrator> semantic_integrator_;
};

void declare_config(ProjectiveIntegrator::Config& config);

}  // namespace hydra
