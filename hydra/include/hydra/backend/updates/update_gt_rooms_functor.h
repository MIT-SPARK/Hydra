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
#include <spark_dsg/bounding_box.h>

#include <filesystem>
#include <vector>

#include "hydra/backend/update_functions.h"
#include "hydra/common/output_sink.h"

namespace hydra {

struct RoomExtents {
  using BoundingBoxes = std::vector<std::vector<spark_dsg::BoundingBox>>;

  explicit RoomExtents(const BoundingBoxes& boxes);
  explicit RoomExtents(const std::filesystem::path& path_to_yaml);

  struct QueryResult {
    bool valid = false;
    size_t index = 0;
  };
  QueryResult getRoomForPoint(Eigen::Vector3d point) const;

  BoundingBoxes room_bounding_boxes;
};

struct UpdateGtRoomsFunctor : public UpdateFunctor {
  using Sink = OutputSink<uint64_t, const RoomExtents&>;
  struct Config {
    std::filesystem::path ground_truth_rooms_path;
    std::string places_layer = spark_dsg::DsgLayers::PLACES;
    char room_prefix = 'R';
    std::vector<Sink::Factory> sinks;
  } const config;

  explicit UpdateGtRoomsFunctor(const Config& config);

  void call(const spark_dsg::SceneGraph& /* unmerged */,
            SharedDsgInfo& dsg,
            const UpdateInfo::ConstPtr& info) const override;

 private:
  const RoomExtents room_extents_;
  const Sink::List sinks_;
};

void declare_config(UpdateGtRoomsFunctor::Config& config);

}  // namespace hydra
