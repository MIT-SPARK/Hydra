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

#include <spark_dsg/node_symbol.h>

#include <Eigen/Geometry>
#include <cstdint>
#include <string>

namespace hydra {

//! Default partition (and node symbol category) of the agents layer for sub-keyframe
//! nodes, distinct from the robot prefixes (`a`-`h`) and the kimera_pgmo vertex
//! prefixes (`s`-`z`)
inline constexpr char kDefaultSubKeyframePartition = 'k';

//! @brief Node id of the index-th sub-keyframe of a robot. The robot id is stored in
//! the upper bits of the symbol index so that ids are unique across robots
inline spark_dsg::NodeId subKeyframeNodeId(char category, int robot_id, size_t index) {
  return spark_dsg::NodeSymbol(category, (static_cast<size_t>(robot_id) << 48) | index);
}

//! Request for a sub-keyframe node, created by the frontend once anchors exist
struct SubKeyframeRequest {
  uint64_t timestamp_ns = 0;
  //! Body pose of the sub-keyframe
  Eigen::Isometry3d world_T_subframe = Eigen::Isometry3d::Identity();
  //! Image folder value of the sub-keyframe images (see KeyframeWriter::imageFolder)
  std::string image_folder;
};

}  // namespace hydra
