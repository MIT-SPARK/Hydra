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
#include "hydra/frontend/gvd_place_extractor.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

#include "hydra/common/global_info.h"
#include "hydra/places/graph_extractor_interface.h"
#include "hydra/places/graph_extractor_utilities.h"
#include "hydra/places/gvd_integrator.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ScopedTimer;
using places::GvdIntegrator;
using places::GvdIntegratorConfig;
using places::GvdVoxel;

void declare_config(GvdPlaceExtractor::Config& config) {
  using namespace config;
  name("GvdPlaceExtractor::Config");
  field(config.gvd, "gvd");
  config.graph.setOptional();
  field(config.graph, "graph");
  config.tsdf_interpolator.setOptional();
  field(config.tsdf_interpolator, "tsdf_interpolator");
  field(config.min_component_size, "min_component_size");
  field(config.filter_places, "filter_places");
  field(config.filter_ground, "filter_ground");
  field(config.robot_height, "robot_height");
  field(config.edge_tolerance, "edge_tolerance");
  field(config.node_tolerance, "node_tolerance");
  field(config.add_freespace_edges, "add_freespace_edges");
  if (config.add_freespace_edges) {
    // TODO(nathan) see why ADL is broken
    field(config.freespace_config.max_length_m, "max_length_m");
    field(config.freespace_config.num_nodes_to_check, "num_nodes_to_check");
    field(config.freespace_config.num_neighbors_to_find, "num_neighbors_to_find");
    field(config.freespace_config.min_clearance_m, "min_clearance_m");
  }
  field(config.sinks, "sinks");
}

GvdPlaceExtractor::GvdPlaceExtractor(const Config& c)
    : config(config::checkValid(c)),
      graph_extractor_(config.graph.create()),
      sinks_(Sink::instantiate(config.sinks)) {
  if (!graph_extractor_) {
    LOG(ERROR) << "no place graph extraction provided! disabling extraction";
  }

  tsdf_interpolator_ = config.tsdf_interpolator.create();
  if (tsdf_interpolator_) {
    LOG(INFO) << "Downsampling TSDF when creating places!";
  }

  const auto& map_config = GlobalInfo::instance().getMapConfig();
  if (static_cast<float>(config.gvd.min_distance_m) >= map_config.truncation_distance) {
    LOG(ERROR)
        << "integrator min distance must be less than truncation distance (currently "
        << config.gvd.min_distance_m << " vs. truncation distance "
        << map_config.truncation_distance << ")";
    throw std::runtime_error("invalid integrator min distance");
  }

  size_t sink_idx = 0;
  for (const auto& sink : sinks_) {
    VLOG(1) << "Sink " << sink_idx << ": " << (sink ? sink->printInfo() : "n/a");
    ++sink_idx;
  }
}

GvdPlaceExtractor::~GvdPlaceExtractor() {}

void GvdPlaceExtractor::save(const LogSetup& log_setup) const {
  const auto output_path = log_setup.getLogDir("frontend");
  if (graph_extractor_) {
    const auto& original_places = graph_extractor_->getGraph();
    auto places = original_places.clone();

    std::unique_ptr<DynamicSceneGraph::Edges> edges(new DynamicSceneGraph::Edges());
    for (const auto& id_edge_pair : places->edges()) {
      edges->emplace(std::piecewise_construct,
                     std::forward_as_tuple(id_edge_pair.first),
                     std::forward_as_tuple(id_edge_pair.second.source,
                                           id_edge_pair.second.target,
                                           id_edge_pair.second.info->clone()));
    }

    DynamicSceneGraph::Ptr graph(new DynamicSceneGraph());
    graph->updateFromLayer(*places, std::move(edges));
    graph->save(output_path + "/places.json", false);
  }
}

NodeIdSet GvdPlaceExtractor::getActiveNodes() const { return active_nodes_; }

std::vector<bool> GvdPlaceExtractor::inFreespace(const PositionMatrix& positions,
                                                 double freespace_distance_m) const {
  if (positions.cols() < 1) {
    return {};
  }

  std::vector<bool> flags(positions.cols(), false);
  // starting lock on tsdf update
  std::unique_lock<std::mutex> lock(gvd_mutex_);
  for (int i = 0; i < positions.cols(); ++i) {
    const Point position = positions.col(i).cast<Point::Scalar>();
    const auto* voxel = gvd_->getVoxelPtr(position);
    if (!voxel) {
      continue;
    }

    flags[i] = voxel->observed && voxel->distance > freespace_distance_m;
  }

  return flags;
}

void GvdPlaceExtractor::detect(const ReconstructionOutput& msg) {
  ScopedTimer timer("frontend/detect_gvd", msg.timestamp_ns, true, 2, false);

  const auto& map = msg.map();
  const auto* tsdf = &map.getTsdfLayer();

  TsdfLayer::Ptr tsdf_ptr;
  if (tsdf_interpolator_) {
    ScopedTimer dtimer("frontend/downsample_tsdf", msg.timestamp_ns, true, 2, false);
    const auto blocks = tsdf->blockIndicesWithCondition(TsdfBlock::esdfUpdated);
    tsdf_ptr = tsdf_interpolator_->interpolate(*tsdf, &blocks);
    for (auto& block : *tsdf_ptr) {
      block.setUpdated();
    }

    tsdf = tsdf_ptr.get();
  }

  if (!gvd_) {
    gvd_.reset(new places::GvdLayer(tsdf->voxel_size, tsdf->voxels_per_side));
    gvd_integrator_.reset(new GvdIntegrator(config.gvd, gvd_, graph_extractor_));
  }

  const Eigen::Isometry3f world_T_body = msg.world_T_body().cast<float>();
  latest_pos_ = world_T_body.translation().cast<double>();

  {  // start critical section
    std::unique_lock<std::mutex> lock(gvd_mutex_);
    ScopedTimer timer("places/gvd", msg.timestamp_ns);
    gvd_integrator_->updateFromTsdf(msg.timestamp_ns, *tsdf, true);
    gvd_integrator_->updateGvd(msg.timestamp_ns);
    gvd_integrator_->archiveBlocks(msg.archived_blocks);
  }  // end critical section

  Sink::callAll(sinks_, msg.timestamp_ns, world_T_body, *gvd_, graph_extractor_.get());
}

void filterInvalidNodes(const SceneGraphLayer& graph, NodeIdSet& active_nodes) {
  std::list<NodeId> invalid_nodes;
  for (const NodeId active_id : active_nodes) {
    const auto& node = graph.getNode(active_id);
    const auto& attrs = node.attributes<PlaceNodeAttributes>();
    if (std::isnan(attrs.distance)) {
      invalid_nodes.push_back(active_id);
      continue;
    }

    if (attrs.position.hasNaN()) {
      invalid_nodes.push_back(active_id);
      continue;
    }

    for (const auto& info : attrs.voxblox_mesh_connections) {
      if (std::isnan(info.voxel_pos[0]) || std::isnan(info.voxel_pos[1]) ||
          std::isnan(info.voxel_pos[2])) {
        invalid_nodes.push_back(active_id);
        continue;
      }
    }
  }

  if (!invalid_nodes.empty()) {
    LOG(ERROR) << "Nodes found with invalid attributes: "
               << displayNodeSymbolContainer(invalid_nodes);
    for (const auto& invalid_id : invalid_nodes) {
      active_nodes.erase(invalid_id);
    }
  }
}

void GvdPlaceExtractor::updateGraph(uint64_t timestamp_ns, DynamicSceneGraph& graph) {
  ScopedTimer timer("frontend/update_gvd_places", timestamp_ns, true, 2, false);
  if (!graph_extractor_) {
    return;
  }

  active_nodes_ = graph_extractor_->getActiveNodes();
  const auto& places = graph_extractor_->getGraph();
  filterInvalidNodes(places, active_nodes_);
  VLOG(2) << "[Hydra Frontend] Considering " << active_nodes_.size()
          << " input place nodes ";

  NodeIdSet active_neighborhood = active_nodes_;
  for (const auto& node_id : graph_extractor_->getDeletedNodes()) {
    const auto node = graph.findNode(node_id);
    if (!node) {
      continue;
    }

    const auto& siblings = node->siblings();
    active_neighborhood.insert(siblings.begin(), siblings.end());
    graph.removeNode(node_id);
  }

  const auto& deleted_edges = graph_extractor_->getDeletedEdges();
  for (size_t i = 0; i < deleted_edges.size(); i += 2) {
    const auto n1 = deleted_edges.at(i);
    const auto n2 = deleted_edges.at(i + 1);
    active_neighborhood.insert(n1);
    active_neighborhood.insert(n2);
    graph.removeEdge(n1, n2);
  }

  for (const auto node_id : active_nodes_) {
    const auto& node = places.getNode(node_id);
    auto attrs = node.attributes().clone();
    attrs->is_active = true;
    attrs->last_update_time_ns = timestamp_ns;
    graph.addOrUpdateNode(DsgLayers::PLACES, node_id, std::move(attrs));

    for (const auto sibling : node.siblings()) {
      const auto& edge = places.getEdge(node_id, sibling);
      graph.addOrUpdateEdge(edge.source, edge.target, edge.info->clone());
    }
  }

  if (config.filter_ground) {
    filterGround(graph);
  }

  if (config.filter_places) {
    filterIsolated(graph, active_neighborhood);
  }

  graph_extractor_->clearDeleted();
}

void GvdPlaceExtractor::filterIsolated(DynamicSceneGraph& graph,
                                       NodeIdSet& active_neighborhood) {
  const auto& places = graph_extractor_->getGraph();

  auto iter = active_neighborhood.begin();
  while (iter != active_neighborhood.end()) {
    if (places.hasNode(*iter)) {
      ++iter;
      continue;
    }

    iter = active_neighborhood.erase(iter);
  }

  // we grab connected components using the subgraph of all active places and all
  // archived places that used to be a neighbor with an active place so that we don't
  // miss disconnected components that comprised of archived nodes and formed when an
  // active node or edge is removed. Limiting the connected component search to be
  // within N hops of the subgraph, where N is the min allowable component size
  // ensures that we don't search the entire places subgraph, but still preserve
  // archived places that connect to a component of at least size N
  const auto components =
      graph_utilities::getConnectedComponents(graph.getLayer(DsgLayers::PLACES),
                                              config.min_component_size,
                                              active_neighborhood);

  for (const auto& component : components) {
    if (component.size() >= config.min_component_size) {
      continue;
    }

    for (const auto to_delete : component) {
      graph.removeNode(to_delete);
      active_nodes_.erase(to_delete);
    }
  }
}

void GvdPlaceExtractor::filterGround(DynamicSceneGraph& graph) {
  const double max_z = latest_pos_.z() - config.robot_height + config.node_tolerance;
  std::list<NodeId> invalid_nodes;
  std::list<EdgeKey> invalid_edges;
  for (const auto& node_id : active_nodes_) {
    const auto& node = graph.getNode(node_id);
    const auto& attrs = node.attributes<PlaceNodeAttributes>();
    double curr_min_z = attrs.position.z() - attrs.distance;
    if (curr_min_z > max_z) {
      // invalid nodes will be removed (and remove all edges), so no need to check edges
      invalid_nodes.push_back(node_id);
      continue;
    }

    for (const auto& sibling : node.siblings()) {
      // this check should work: each side of the edge will be one of the extreme values
      // (and each side should be visited once
      const auto& edge = graph.getEdge(node_id, sibling);
      double edge_min_z = attrs.position.z() - edge.attributes().weight;
      if (edge_min_z > max_z) {
        invalid_edges.push_back(EdgeKey(edge.source, edge.target));
      }
    }
  }

  VLOG(5) << "Erasing " << invalid_nodes.size() << " nodes and " << invalid_edges.size()
          << " edges that do not reach ground-plane";
  for (const auto node_id : invalid_nodes) {
    graph.removeNode(node_id);
    active_nodes_.erase(node_id);
  }

  for (const auto edge_key : invalid_edges) {
    graph.removeEdge(edge_key.k1, edge_key.k2);
  }

  places::EdgeInfoMap new_edges;
  places::findFreespaceEdges(config.freespace_config,
                             graph.getLayer(DsgLayers::PLACES),
                             *gvd_,
                             active_nodes_,
                             graph_extractor_->getIndexMap(),
                             new_edges);
  for (auto&& [edge_key, attrs] : new_edges) {
    const auto& source_pos = graph.getPosition(edge_key.k1);
    const auto& target_pos = graph.getPosition(edge_key.k2);
    double source_min_z = source_pos.z() - attrs->weight;
    double target_min_z = target_pos.z() - attrs->weight;
    if (source_min_z > max_z || target_min_z > max_z) {
      continue;
    }

    graph.insertEdge(edge_key.k1, edge_key.k2, std::move(attrs));
  }
}

}  // namespace hydra
