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

#include <cstdint>
#include <list>
#include <map>
#include <set>
#include <unordered_map>

namespace hydra::places {

struct CompressedNode {
  using CompressedNodeMap = std::unordered_map<uint64_t, CompressedNode>;
  uint64_t node_id;
  std::set<uint64_t> siblings;
  std::set<uint64_t> active_refs;
  std::set<uint64_t> archived_refs;
  std::unordered_map<uint64_t, std::map<uint64_t, uint64_t>> sibling_support;
  std::map<uint64_t, uint64_t> sibling_ref_counts;
  uint64_t best_gvd_id;
  bool in_graph = false;

  explicit CompressedNode(uint64_t node_id);

  void addEdgeObservation(uint64_t gvd_id,
                          uint64_t neighbor_gvd_id,
                          uint64_t sibling_id);

  bool removeEdgeObservation(uint64_t gvd_id, uint64_t neighbor_gvd_id);

  std::list<uint64_t> removeEdgeObservations(uint64_t gvd_id, CompressedNodeMap& nodes);

  void mergeObservations(uint64_t original_id, uint64_t new_id);

  void merge(CompressedNode& other, CompressedNodeMap& nodes);
};

}  // namespace hydra::places
