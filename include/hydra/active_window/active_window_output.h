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
#include "hydra/input/input_packet.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

struct ActiveWindowOutput {
  using Ptr = std::shared_ptr<ActiveWindowOutput>;

  ActiveWindowOutput() = default;
  virtual ~ActiveWindowOutput() = default;

  //! Timestamp of update
  uint64_t timestamp_ns;
  //! Translation component of body pose
  Eigen::Vector3d world_t_body;
  //! Rotation component of body pose
  Eigen::Quaterniond world_R_body;
  //! Sensor data from last update
  std::shared_ptr<InputData> sensor_data;
  //! New nodes to add to the scene graph
  GraphUpdate graph_update;
  //! Archived mesh blocks on this pass
  spatial_hash::BlockIndices archived_mesh_indices;

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

  /**
   * @brief Construct an output packet from the input to the active window
   */
  static Ptr fromInput(const InputPacket& input);

  /*
   * @brief Get the body pose from when this packet was created
   */
  template <typename T = double>
  Eigen::Transform<T, 3, Eigen::Isometry> world_T_body() const {
    return Eigen::Translation<T, 3>(world_t_body.cast<T>()) * world_R_body.cast<T>();
  }

 protected:
  std::shared_ptr<VolumetricMap> map_;
};

}  // namespace hydra
