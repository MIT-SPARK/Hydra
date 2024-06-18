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
#include "hydra/places/gvd_graph.h"

namespace hydra::places {

GvdGraph::GvdGraph() : next_id_(0) {}

bool GvdGraph::empty() const { return nodes_.empty(); }

uint64_t GvdGraph::addNode(const Eigen::Vector3d& position, const GlobalIndex& index) {
  // position can't change ever (so we only ever set it when adding)
  GvdMemberInfo info;
  info.position = position;
  info.index = index;

  const auto next_id = getNextId();
  nodes_.emplace(next_id, info);
  return next_id;
}

void GvdGraph::removeNode(uint64_t node) {
  auto iter = nodes_.find(node);
  if (iter == nodes_.end()) {
    return;
  }

  for (const auto sibling_id : iter->second.siblings) {
    nodes_.at(sibling_id).siblings.erase(node);
  }

  id_queue_.push_back(node);
  nodes_.erase(iter);
}

const GvdMemberInfo* GvdGraph::getNode(uint64_t node) const {
  return const_cast<GvdGraph*>(this)->getNode(node);
}

GvdMemberInfo* GvdGraph::getNode(uint64_t node) {
  // TODO(nathan) make this not throw out-of-range
  return &nodes_.at(node);
}

const GvdGraph::Nodes& GvdGraph::nodes() const { return nodes_; }

uint64_t GvdGraph::getNextId() {
  uint64_t new_id;
  if (id_queue_.empty()) {
    new_id = next_id_;
    next_id_++;
  } else {
    new_id = id_queue_.front();
    id_queue_.pop_front();
  }

  return new_id;
}

bool GvdGraph::hasNode(uint64_t node) const { return nodes_.count(node) > 0; }

}  // namespace hydra::places
