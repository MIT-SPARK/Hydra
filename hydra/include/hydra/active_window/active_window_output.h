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
#include <spark_dsg/node_attributes.h>

#include <Eigen/Dense>
#include <cstdint>
#include <memory>

#include "hydra/common/graph_update.h"
#include "hydra/input/input_data.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

// Batches archive blocks before applying replacements. Payloads are immutable
// while an output is in use; only the latest update in each lifetime is retained.
struct MapUpdateBatch {
  uint64_t timestamp_ns = 0;
  spatial_hash::BlockIndices archived;
  std::vector<TsdfBlock::ConstPtr> tsdf;
  std::vector<MeshBlock::ConstPtr> mesh;
  std::vector<SemanticBlock::ConstPtr> semantic;
  std::vector<TrackingBlock::ConstPtr> tracking;

  bool empty() const;
};

// Compatibility view for consumers that only need mesh updates.
struct MeshUpdateBatch {
  uint64_t timestamp_ns = 0;
  spatial_hash::BlockIndices archived;
  std::vector<MeshBlock::ConstPtr> blocks;
};

struct ActiveWindowOutput {
  using Ptr = std::shared_ptr<ActiveWindowOutput>;

  ActiveWindowOutput() = default;
  virtual ~ActiveWindowOutput() = default;

  //! Ordered changes for every volumetric layer, including uncollated outputs.
  std::vector<MapUpdateBatch> mapUpdates() const;

  //! Ordered mesh changes projected from mapUpdates().
  std::vector<MeshUpdateBatch> meshUpdates() const;

  //! Timestamp of update
  uint64_t timestamp_ns = 0;
  //! Sensor data from last update
  InputData::ConstPtr sensor_data;
  //! New nodes to add to the scene graph
  GraphUpdate graph_update;
  //! Archived blocks on this pass
  spatial_hash::BlockIndices archived;

  /**
   * @brief Get the current volumetric map
   */
  const VolumetricMap& map() const;

  /**
   * @brief Sets the output volumetric map (by copying the input)
   */
  void setMap(const VolumetricMap& map);

  /**
   * @brief Sets the output volumetric map (without copying)
   */
  void setMap(const std::shared_ptr<VolumetricMap>& map);

  /**
   * @brief Collate the other active window output into this
   * @param msg Message to add to this one (is invalid after this call)
   * @param clone_map Explicitly copy the other map if the map for this message is not
   * set
   */
  virtual void updateFrom(ActiveWindowOutput&& msg, bool clone_map);

  //! Whether final-map consumers can process both messages in a single pass.
  bool canCollate(const ActiveWindowOutput& msg) const;

  /*
   * @brief Get the body pose from when this packet was created
   */
  template <typename T = double>
  Eigen::Transform<T, 3, Eigen::Isometry> world_T_body() const {
    return sensor_data->world_T_body.cast<T>();
  }

 protected:
  std::shared_ptr<VolumetricMap> map_;
  void updateMap(const ActiveWindowOutput& msg, bool clone_map);
  void appendGraphUpdate(GraphUpdate& update);

  std::vector<MapUpdateBatch> map_updates_;
};

}  // namespace hydra
