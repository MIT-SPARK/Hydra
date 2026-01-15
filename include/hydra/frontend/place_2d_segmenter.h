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
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/node_symbol.h>

#include "hydra/active_window/active_window_output.h"
#include "hydra/common/output_sink.h"
#include "hydra/frontend/mesh_delta_clustering.h"
#include "hydra/utils/logging.h"

namespace kimera_pgmo {
class MeshDelta;
struct MeshOffsetInfo;
}  // namespace kimera_pgmo

namespace hydra {

using clustering::LabelIndices;

class Place2dSegmenter {
 public:
  using Sink = OutputSink<uint64_t,
                          const kimera_pgmo::MeshDelta&,
                          const kimera_pgmo::MeshOffsetInfo&>;

  struct Config : VerbosityConfig {
    Config();

    std::string layer = spark_dsg::DsgLayers::MESH_PLACES;
    char prefix = 'Q';
    clustering::ClusteringConfig clustering{1.0, 600, 100000};
    double pure_final_place_size = 3;
    size_t min_final_place_points = 1000;
    double place_overlap_threshold = 0.1;
    double place_max_neighbor_z_diff = 0.5;
    double connection_ellipse_scale_factor = 1;
    std::vector<Sink::Factory> sinks;
  } const config;

  Place2dSegmenter(const Config& config, const std::set<uint32_t>& labels);

  void detect(const ActiveWindowOutput& msg,
              const kimera_pgmo::MeshDelta& mesh_delta,
              const kimera_pgmo::MeshOffsetInfo& offsets);

  void updateGraph(const ActiveWindowOutput& msg,
                   const kimera_pgmo::MeshOffsetInfo& offsets,
                   spark_dsg::DynamicSceneGraph& graph);

 private:
  Sink::List sinks_;
  std::set<uint32_t> labels_;
  spark_dsg::NodeSymbol next_node_id_;
  std::list<spark_dsg::NodeId> to_remove_;
  std::map<spark_dsg::NodeId, spark_dsg::Place2dNodeAttributes> active_places_;
};

void declare_config(Place2dSegmenter::Config& config);

}  // namespace hydra
