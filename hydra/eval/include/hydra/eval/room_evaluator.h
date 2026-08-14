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

#include <list>
#include <map>
#include <memory>
#include <string>

#include "hydra/eval/room_io.h"
#include "hydra/eval/room_metrics.h"
#include "hydra/reconstruction/voxel_types.h"

namespace hydra::eval {

class RoomEvaluator {
 public:
  using Ptr = std::unique_ptr<RoomEvaluator>;

  struct Config {
    bool only_labeled = false;
    float min_weight = 1.0e-6f;
    float min_distance = 0.0f;
    size_t min_room_nodes = 0;
  } const config;

  RoomEvaluator(const Config& config,
                const RoomGeometry& rooms,
                const TsdfLayer::Ptr& tsdf);

  void computeRoomIndices();

  const RoomIndices& getRoomIndices() const;

  void computeDsgIndices(const DynamicSceneGraph& graph, RoomIndices& indices) const;

  RoomMetrics eval(const std::string& graph_filepath) const;

  static GlobalIndices getSphereAroundPoint(const TsdfLayer& layer,
                                            const Point& center,
                                            float radius);

 public:
  static RoomEvaluator::Ptr fromFile(const Config& config,
                                     const std::string& room_filepath,
                                     const std::string& tsdf_filepath);

 private:
  RoomGeometry rooms_;
  TsdfLayer::Ptr tsdf_;
  RoomIndices room_indices_;
};

}  // namespace hydra::eval
