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
#include <spark_dsg/scene_graph.h>

#include <fstream>

#include "hydra/rooms/graph_clustering.h"
#include "hydra/rooms/room_finder_config.h"
#include "hydra/rooms/room_utilities.h"

namespace hydra {

struct DistanceAdaptor;

struct GraphInfo {
  size_t offset;
  size_t size;
  size_t num_nodes;
  size_t num_edges;
};

class RoomFinder {
 public:
  using Layer = spark_dsg::SceneGraphLayer;
  using ClusterMap = std::map<spark_dsg::NodeId, std::vector<spark_dsg::NodeId>>;

  explicit RoomFinder(const RoomFinderConfig& config);

  virtual ~RoomFinder();

  Layer::Ptr findRooms(const Layer& places);

  void addRoomPlaceEdges(spark_dsg::SceneGraph& graph, const std::string& layer) const;

  void enableLogging(const std::string& log_path);

  void fillClusterMap(const Layer& places, ClusterMap& assignments) const;

  const RoomFinderConfig config;

  const RoomExtents room_extents;

 protected:
  InitialClusters getBestComponents(const Layer& places) const;

  Layer::Ptr makeRoomLayer(const Layer& places);

  void setupDistanceAdaptor(const Layer& places);

  std::unique_ptr<DistanceAdaptor> distance_adaptor_;
  ClusterResults last_results_;
  std::map<size_t, spark_dsg::NodeId> cluster_room_map_;
  mutable bool logged_once_ = false;
  std::unique_ptr<std::ofstream> log_file_;
  std::unique_ptr<std::ofstream> graph_log_file_;
  mutable std::vector<GraphInfo> graph_entries_;
  mutable size_t graph_offset_ = 0;
};

}  // namespace hydra
