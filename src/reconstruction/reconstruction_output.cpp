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
#include "hydra/reconstruction/reconstruction_output.h"

namespace hydra {

using OutputPtr = ReconstructionOutput::Ptr;

const VolumetricMap& ReconstructionOutput::map() const {
  CHECK(map_) << "Invalid map!";
  return *map_;
}

void ReconstructionOutput::updateFrom(const ReconstructionOutput& msg, bool clone_map) {
  timestamp_ns = msg.timestamp_ns;
  world_t_body = msg.world_t_body;
  world_R_body = msg.world_R_body;
  sensor_data = msg.sensor_data;

  archived_blocks.insert(
      archived_blocks.end(), msg.archived_blocks.begin(), msg.archived_blocks.end());

  if (!msg.map_) {
    LOG(ERROR) << "Reconstruction output message contained no map!";
    return;
  }

  if (!map_ && !clone_map) {
    // avoid copying the first map if possible
    map_ = msg.map_;
  }

  const auto& new_map = *msg.map_;
  if (!map_) {
    // make a new map if we don't have one and we are forcing a clone
    const auto has_labels = new_map.getSemanticLayer() != nullptr;
    map_ = std::make_shared<VolumetricMap>(new_map.config, has_labels);
  }

  map_->updateFrom(new_map);
}

void ReconstructionOutput::setMap(const VolumetricMap& map) {
  auto new_map = map.clone();
  map_.reset(new_map.release());
}

void ReconstructionOutput::setMap(const std::shared_ptr<VolumetricMap>& map) {
  map_ = map;
}

std::shared_ptr<VolumetricMap> ReconstructionOutput::getMapPointer() const {
  return map_;
}

OutputPtr ReconstructionOutput::fromInput(const InputPacket& msg) {
  auto new_msg = std::make_shared<ReconstructionOutput>();
  new_msg->timestamp_ns = msg.timestamp_ns;
  new_msg->world_t_body = msg.world_t_body;
  new_msg->world_R_body = msg.world_R_body;
  return new_msg;
}

}  // namespace hydra
