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
#include "hydra/common/dsg_types.h"
#include "hydra/utils/disjoint_set.h"

namespace hydra {

struct FiltrationInfo {
  double distance;
  size_t num_components;
};

struct ComponentLifetime {
  double start;
  double end;
};

using LifetimeMap = std::unordered_map<NodeId, ComponentLifetime>;

struct BarcodeTracker : public DisjointSet {
  BarcodeTracker();

  explicit BarcodeTracker(size_t min_component_size);

  virtual ~BarcodeTracker() = default;

  void addNode(NodeId node, double distance);

  bool doUnion(DisjointSet& components,
               const std::unordered_map<NodeId, double>& node_distances,
               NodeId node,
               NodeId rhs,
               double distance);

  size_t min_component_size;
  LifetimeMap barcodes;
};

using Filtration = std::vector<FiltrationInfo>;
using ComponentCallback = std::function<size_t(const DisjointSet&)>;

std::ostream& operator<<(std::ostream& out, const FiltrationInfo& info);

std::ostream& operator<<(std::ostream& out, const Filtration& info);

Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              double diff_threshold_m = 1.0e-4);

Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              size_t min_component_size,
                              double diff_threshold_m = 1.0e-4);

Filtration getGraphFiltration(const SceneGraphLayer& layer,
                              BarcodeTracker& tracker,
                              double diff_threshold_m,
                              const ComponentCallback& count_components,
                              bool include_nodes = true);

std::pair<size_t, size_t> getTrimmedFiltration(const Filtration& old_filtration,
                                               double min_dilation_m,
                                               double max_dilation_m,
                                               bool clip_to_max = true);

std::optional<FiltrationInfo> getLongestSequence(const Filtration& values,
                                                 size_t start_index,
                                                 size_t end_index);

std::optional<FiltrationInfo> getLongestLifetimeDilation(const Filtration& values,
                                                         const LifetimeMap& lifetimes,
                                                         double min_component_lifetime,
                                                         size_t start_index,
                                                         size_t end_index);

std::optional<FiltrationInfo> getBestPlateau(const Filtration& values,
                                             double ratio,
                                             size_t start_index,
                                             size_t end_index,
                                             bool use_threshold = false);

}  // namespace hydra
