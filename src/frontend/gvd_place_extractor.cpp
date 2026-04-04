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
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <spark_dsg/graph_utilities.h>
#include <spark_dsg/printing.h>

#include "hydra/common/global_info.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using places::GvdIntegrator;
using spark_dsg::DsgLayers;
using spark_dsg::NodeId;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SceneGraph;
using spark_dsg::SceneGraphLayer;
using spark_dsg::graph_utilities::getConnectedComponents;
using timing::ScopedTimer;

using PlacesGraph = PartialGraph<PlaceNodeAttributes>;

namespace {

static const auto registration =
    config::RegistrationWithConfig<GvdPlaceExtractor,
                                   GvdPlaceExtractor,
                                   GvdPlaceExtractor::Config>("gvd");

std::set<NodeId> getInvalidNodes(const PlacesGraph& graph) {
  std::set<NodeId> invalid_nodes;
  for (const auto& [node_id, node] : graph.nodes()) {
    if (!node.attrs) {
      continue;
    }

    if (std::isnan(node.attrs->distance)) {
      invalid_nodes.insert(node_id);
      continue;
    }

    if (node.attrs->position.hasNaN()) {
      invalid_nodes.insert(node_id);
      continue;
    }

    for (const auto& info : node.attrs->voxblox_mesh_connections) {
      if (std::isnan(info.voxel_pos[0]) || std::isnan(info.voxel_pos[1]) ||
          std::isnan(info.voxel_pos[2])) {
        invalid_nodes.insert(node_id);
        continue;
      }
    }
  }

  if (!invalid_nodes.empty()) {
    LOG(ERROR) << "Nodes found with invalid attributes: "
               << spark_dsg::displayNodeSymbolContainer(invalid_nodes);
  }

  return invalid_nodes;
}

}  // namespace

void declare_config(GvdPlaceExtractor::Config& config) {
  using namespace config;
  name("GvdPlaceExtractor::Config");
  base<VerbosityConfig>(config);
  field(config.layer, "layer");
  field(config.gvd, "gvd");
  field(config.graph, "graph");
  config.tsdf_interpolator.setOptional();
  field(config.tsdf_interpolator, "tsdf_interpolator");
  field(config.min_component_size, "min_component_size");
  field(config.sinks, "sinks");
}

GvdPlaceExtractor::GvdPlaceExtractor(const Config& c)
    : config(config::checkValid(c)),
      graph_extractor_(config.graph),
      map_window_(GlobalInfo::instance().createVolumetricWindow()),
      sinks_(Sink::instantiate(config.sinks)) {
  tsdf_interpolator_ = config.tsdf_interpolator.create();
  if (tsdf_interpolator_) {
    MLOG(0) << "Downsampling TSDF when creating places!";
  }

  MLOG(1) << "\n" << Sink::printSinks(sinks_);
}

GvdPlaceExtractor::~GvdPlaceExtractor() {}

NodeIdSet GvdPlaceExtractor::getActiveNodes() const { return active_nodes_; }

void GvdPlaceExtractor::detect(const ActiveWindowOutput& msg) {
  ScopedTimer timer("frontend/detect_gvd", msg.timestamp_ns, true, 2, false);

  const auto& map = msg.map();
  if (static_cast<float>(config.gvd.min_distance_m) >= map.config.truncation_distance) {
    LOG(ERROR) << "GVD integrator min distance must be less than truncation distance "
                  "(currently "
               << config.gvd.min_distance_m << " vs. truncation distance "
               << map.config.truncation_distance << ")";
    return;
  }

  TsdfLayer::Ptr downsampled_tsdf;
  if (tsdf_interpolator_) {
    ScopedTimer dtimer("frontend/downsample_tsdf", msg.timestamp_ns, true, 2, false);
    downsampled_tsdf = tsdf_interpolator_->interpolate(map.getTsdfLayer());
  }

  const auto& tsdf = downsampled_tsdf ? *downsampled_tsdf : map.getTsdfLayer();
  const Eigen::Isometry3d world_T_body = msg.world_T_body();

  if (!gvd_) {
    gvd_.reset(new places::GvdLayer(tsdf.voxel_size, tsdf.voxels_per_side));
    gvd_integrator_.reset(new GvdIntegrator(config.gvd, gvd_));
  }

  ScopedTimer graph_timer("places/gvd", msg.timestamp_ns);
  // reconstruction now only sends updated blocks so we integrate everything
  gvd_integrator_->updateFromTsdf(
      msg.timestamp_ns, tsdf, false, &map.getMeshLayer(), true);
  gvd_integrator_->updateGvd(msg.timestamp_ns, &graph_extractor_);

  if (map_window_) {
    BlockIndices to_archive;
    for (const auto& block : *gvd_) {
      if (!map_window_->inBounds(msg.timestamp_ns, world_T_body, block)) {
        to_archive.push_back(block.index);
      }
    }

    gvd_integrator_->archiveBlocks(to_archive, &graph_extractor_);
  }

  Sink::callAll(sinks_, msg.timestamp_ns, world_T_body, *gvd_, graph_extractor_);
}

void GvdPlaceExtractor::updateGraph(uint64_t timestamp_ns, SceneGraph& graph) {
  ScopedTimer timer("frontend/update_gvd_places", timestamp_ns, true, 2, false);

  const auto& places = graph_extractor_.graph();
  MLOG(1) << "Considering " << places.nodes().size() << " input place nodes ";

  std::vector<NodeId> deleted_nodes;
  // const auto deleted_nodes = graph_extractor_.getDeletedNodes()
  for (const auto& node_id : deleted_nodes) {
    const auto node = graph.findNode(node_id);
    if (!node) {
      continue;
    }

    // TODO(nathan) think about expanding siblings
    graph.removeNode(node_id);
  }

  // TODO(nathan) fix this
  // const auto& deleted_edges = graph_extractor_.getDeletedEdges();
  std::vector<NodeId> deleted_edges;
  for (size_t i = 0; i < deleted_edges.size(); i += 2) {
    const auto n1 = deleted_edges.at(i);
    const auto n2 = deleted_edges.at(i + 1);
    graph.removeEdge(n1, n2);
  }

  NodeIdSet active_neighborhood;
  const auto invalid = getInvalidNodes(places);
  for (const auto& [node_id, node] : places.nodes()) {
    if (invalid.count(node_id)) {
      continue;
    }

    active_neighborhood.insert(node_id);
    auto attrs = node.attrs->clone();
    attrs->is_active = true;
    attrs->last_update_time_ns = timestamp_ns;
    graph.addOrUpdateNode(config.layer, node_id, std::move(attrs));
    for (const auto sibling : places.neighbors(node_id)) {
      graph.addOrUpdateEdge(node_id, sibling, places.at(node_id, sibling)->clone());
    }
  }

  filterIsolated(graph, active_neighborhood);
}

void GvdPlaceExtractor::filterIsolated(SceneGraph& graph,
                                       NodeIdSet& active_neighborhood) {
  if (config.min_component_size < 2) {
    return;  // no need to do any work
  }

  // we grab connected components using the subgraph of all active places and all
  // archived places that used to be a neighbor with an active place so that we don't
  // miss disconnected components that comprised of archived nodes and formed when an
  // active node or edge is removed. Limiting the connected component search to be
  // within N hops of the subgraph, where N is the min allowable component size
  // ensures that we don't search the entire places subgraph, but still preserve
  // archived places that connect to a component of at least size N
  const auto components = getConnectedComponents(graph.getLayer(DsgLayers::PLACES),
                                                 config.min_component_size,
                                                 active_neighborhood);

  for (const auto& component : components) {
    if (component.size() >= config.min_component_size) {
      continue;
    }

    for (const auto to_delete : component) {
      graph.removeNode(to_delete);
    }
  }
}

}  // namespace hydra
