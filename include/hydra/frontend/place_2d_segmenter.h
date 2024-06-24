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
#include <config_utilities/factory.h>
#include <kimera_pgmo/mesh_delta.h>
#include <pcl/common/centroid.h>

#include <memory>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/place_2d_split_logic.h"
#include "hydra/frontend/surface_places_interface.h"

namespace hydra {

class Place2dSegmenter : public SurfacePlacesInterface {
 public:
  using IndicesVector = pcl::IndicesPtr::element_type;
  using LabelIndices = std::map<uint32_t, pcl::IndicesPtr>;
  using MeshVertexCloud = Place2d::CloudT;
  using Places = std::vector<Place2d>;
  using LabelPlaces = std::map<uint32_t, Places>;

  struct Config {
    char prefix = 'Q';
    double cluster_tolerance = 1;
    size_t min_cluster_size = 600;
    size_t max_cluster_size = 100000;
    double pure_final_place_size = 3;
    size_t min_final_place_points = 1000;
    double place_overlap_threshold = 0.1;
    double place_max_neighbor_z_diff = 0.5;
    double connection_ellipse_scale_factor = 1;
    std::set<uint32_t> labels;
  } const config;

  explicit Place2dSegmenter(const Config& config);

  void detect(const ReconstructionOutput& msg,
              const kimera_pgmo::MeshDelta& mesh_delta,
              const DynamicSceneGraph& graph) override;

  NodeIdSet getActiveNodes() const override;

  void updateGraph(uint64_t timestamp_ns,
                   const ReconstructionOutput&,
                   DynamicSceneGraph& graph) override;

 private:
  bool frontendAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                                  const Place2dNodeAttributes& attrs2,
                                  EdgeAttributes& edge_weight);

  Places findPlaces(const Mesh::Positions& positions,
                    const kimera_pgmo::MeshDelta& delta,
                    const pcl::IndicesPtr& indices,
                    double connection_ellipse_scale_factor) const;

  std::set<NodeId> archiveOldObjects(const DynamicSceneGraph& graph,
                                     uint64_t latest_timestamp);

  LabelIndices getLabelIndices(const Mesh::Labels& labels,
                               const IndicesVector& indices) const;

  NodeSymbol addPlaceToGraph(DynamicSceneGraph& graph,
                             const Place2d& place,
                             uint32_t label,
                             uint64_t timestamp);

  void updatePlaceInGraph(const Place2d& place,
                          const SceneGraphNode& node,
                          uint64_t timestamp);

 private:
  LabelPlaces detected_label_places_;

  NodeSymbol next_node_id_;

  std::list<NodeId> nodes_to_remove_;

  size_t num_archived_vertices_;
  std::map<uint32_t, std::set<NodeId>> active_places_;
  std::map<uint32_t, std::set<NodeId>> semiactive_places_;
  std::map<NodeId, uint64_t> active_place_timestamps_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<SurfacePlacesInterface, Place2dSegmenter, Config>(
          "place_2d");
};

void declare_config(Place2dSegmenter::Config& config);

}  // namespace hydra
