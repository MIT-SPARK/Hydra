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
#include <Eigen/Dense>
#include <cstdint>
#include <list>
#include <memory>
#include <set>
#include <unordered_map>

#include "hydra/reconstruction/voxel_types.h"

namespace hydra::places {

struct GvdMemberInfo {
  double distance;
  uint8_t num_basis_points;
  Eigen::Vector3d position;
  GlobalIndex index;
  std::set<uint64_t> siblings;
};

class GvdGraph {
 public:
  using Ptr = std::shared_ptr<GvdGraph>;
  using Nodes = std::unordered_map<uint64_t, GvdMemberInfo>;

  GvdGraph();

  bool empty() const;

  uint64_t addNode(const Eigen::Vector3d& position, const GlobalIndex& index);

  void removeNode(uint64_t node);

  GvdMemberInfo* getNode(uint64_t node);

  const GvdMemberInfo* getNode(uint64_t node) const;

  const Nodes& nodes() const;

  bool hasNode(uint64_t) const;

 protected:
  uint64_t getNextId();

 protected:
  uint64_t next_id_;
  std::list<uint64_t> id_queue_;

  Nodes nodes_;
};

}  // namespace hydra::places
