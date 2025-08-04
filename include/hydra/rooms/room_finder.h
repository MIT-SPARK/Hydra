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
#include <fstream>
#include <unordered_set>

#include "hydra/common/dsg_types.h"
#include "hydra/rooms/graph_clustering.h"

namespace hydra {

struct DistanceAdaptor;

struct GraphInfo {
  size_t offset;
  size_t size;
  size_t num_nodes;
  size_t num_edges;
};

enum class RoomClusterMode { MODULARITY, MODULARITY_DISTANCE, NEIGHBORS, NONE };

enum class DilationThresholdMode {
  REPEATED,
  LONGEST_LIFETIME,
  PLATEAU,
  PLATEAU_THRESHOLD
};

class RoomFinder {
 public:
  using ClusterMap = std::map<NodeId, std::vector<NodeId>>;

  struct Config {
    char room_prefix = 'R';
    double min_dilation_m = 0.1;
    double max_dilation_m = 0.7;
    double min_window_size = 0.2;
    bool clip_dilation_window_to_max = false;
    size_t min_component_size = 15;
    size_t min_room_size = 10;
    DilationThresholdMode dilation_threshold_mode = DilationThresholdMode::REPEATED;
    double min_lifetime_length_m = 0.1;
    double plateau_ratio = 0.5;
    RoomClusterMode clustering_mode = RoomClusterMode::NEIGHBORS;
    double max_modularity_iters = 5;
    double modularity_gamma = 1.0;
    double dilation_diff_threshold_m = 1.0e-4;
    //! @brief Whether or not to log filtration for a particular place graph
    bool log_filtrations = false;
    //! @brief Whether or not to log place graphs
    bool log_place_graphs = false;
    //! @brief Path to log intermediate results. Empty path disables logging
    std::filesystem::path log_path;
  } const config;

  explicit RoomFinder(const Config& config);

  virtual ~RoomFinder();

  SceneGraphLayer::Ptr findRooms(const SceneGraphLayer& places);

  void addRoomPlaceEdges(DynamicSceneGraph& graph, const std::string& layer) const;

 protected:
  InitialClusters getBestComponents(const SceneGraphLayer& places) const;

  SceneGraphLayer::Ptr makeRoomLayer(const SceneGraphLayer& places);

  void setupDistanceAdaptor(const SceneGraphLayer& places);

  std::unique_ptr<DistanceAdaptor> distance_adaptor_;
  ClusterResults last_results_;
  std::map<size_t, NodeId> cluster_room_map_;
  mutable bool logged_once_ = false;
  mutable std::vector<GraphInfo> graph_entries_;
  mutable size_t graph_offset_ = 0;
};

void declare_config(RoomFinder::Config& config);

}  // namespace hydra
