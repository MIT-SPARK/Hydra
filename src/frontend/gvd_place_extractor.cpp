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
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <spark_dsg/graph_utilities.h>
#include <spark_dsg/printing.h>

#include "hydra/common/global_info.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using places::GraphExtractor;
using places::GvdIntegrator;
using spark_dsg::DsgLayers;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::PlaceNodeAttributes;
using spark_dsg::SceneGraph;
using spark_dsg::graph_utilities::getConnectedComponents;
using timing::ScopedTimer;

using PlacesGraph = PartialGraph<PlaceNodeAttributes>;

namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilderFunctor,
                                   GvdPlaceExtractor,
                                   GvdPlaceExtractor::Config>("gvd");

bool attributesInvalid(const PlaceNodeAttributes& attrs) {
  if (std::isnan(attrs.distance)) {
    return true;
  }

  if (attrs.position.hasNaN()) {
    return true;
  }

  for (const auto& info : attrs.voxblox_mesh_connections) {
    if (std::isnan(info.voxel_pos[0]) || std::isnan(info.voxel_pos[1]) ||
        std::isnan(info.voxel_pos[2])) {
      return true;
    }
  }

  return false;
}

}  // namespace

void declare_config(GvdPlaceExtractor::Config& config) {
  using namespace config;
  name("GvdPlaceExtractor::Config");
  base<VerbosityConfig>(config);
  field<CharConversion>(config.node_prefix, "node_prefix");
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
      map_window_(GlobalInfo::instance().createVolumetricWindow()),
      tsdf_interpolator_(config.tsdf_interpolator.create()),
      sinks_(Sink::instantiate(config.sinks)) {
  if (tsdf_interpolator_) {
    MLOG(0) << "Downsampling TSDF when creating places!";
  }

  MLOG(1) << "\n" << Sink::printSinks(sinks_);
}

GvdPlaceExtractor::~GvdPlaceExtractor() {}

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
    gvd_integrator_ = std::make_unique<GvdIntegrator>(config.gvd, gvd_);
    graph_extractor_ = std::make_unique<GraphExtractor>(config.graph, tsdf.voxel_size);
  }

  ScopedTimer gvd_timer("places/gvd", msg.timestamp_ns);
  // reconstruction now only sends updated blocks so we integrate everything
  gvd_integrator_->updateFromTsdf(
      msg.timestamp_ns, tsdf, false, &map.getMeshLayer(), true);

  places::VoxelIndexChanges changes;
  gvd_integrator_->updateGvd(msg.timestamp_ns, &changes);
  graph_extractor_->extract(
      msg.timestamp_ns, *gvd_, changes, gvd_integrator_->parent_tracker());

  if (map_window_) {
    BlockIndices to_archive;
    for (const auto& block : *gvd_) {
      if (!map_window_->inBounds(msg.timestamp_ns, world_T_body, block)) {
        to_archive.push_back(block.index);
      }
    }

    gvd_integrator_->archiveBlocks(to_archive, graph_extractor_.get());
  }

  Sink::callAll(sinks_, msg.timestamp_ns, world_T_body, *gvd_, *graph_extractor_);
}

void GvdPlaceExtractor::updateGraph(uint64_t timestamp_ns, SceneGraph& graph) {
  ScopedTimer timer("frontend/update_gvd_places", timestamp_ns, true, 2, false);
  const auto& places = graph_extractor_->graph();
  MLOG(1) << "Considering " << places.nodes().size() << " input place nodes ";

  for (const auto node_id : places.deleted_nodes()) {
    graph.removeNode(NodeSymbol(config.node_prefix, node_id));
  }

  for (const auto& key : places.deleted_edges()) {
    graph.removeEdge(NodeSymbol(config.node_prefix, key.k1),
                     NodeSymbol(config.node_prefix, key.k2));
  }

  std::unordered_set<uint64_t> to_filter;
  if (config.min_component_size >= 2) {
    // This may delete place nodes that are part of a valid component (ideally we'd keep
    // around the config.min_component_size hops in addition to the latest graph)
    const auto components = places.connected_components(false);
    for (const auto& component : components) {
      if (component.size() < config.min_component_size) {
        to_filter.insert(component.begin(), component.end());
      }
    }
  }

  for (const auto& [node_id, node] : places) {
    const NodeSymbol graph_id(config.node_prefix, node_id);
    const auto& attrs = node.attributes();
    if (attributesInvalid(attrs)) {
      LOG(ERROR) << "Invalid place node " << graph_id.str();
      graph.removeNode(graph_id);
      continue;
    }

    if (to_filter.count(node_id)) {
      if (attrs.is_active) {
        // bad things happen if we delete an archived node, so we only remove active
        // nodes if they were previously added
        MLOG(1) << "Removed isolated node " << graph_id.str();
        graph.removeNode(graph_id);
      }

      continue;
    }

    auto new_attrs = attrs.clone();
    new_attrs->last_update_time_ns = timestamp_ns;
    graph.addOrUpdateNode(config.layer, graph_id, std::move(new_attrs));
  }

  for (const auto& [key, info] : places.edges()) {
    graph.addOrUpdateEdge(NodeSymbol(config.node_prefix, key.k1),
                          NodeSymbol(config.node_prefix, key.k2),
                          info->clone());
  }

  graph_extractor_->prune();  // clear all fully archived nodes
}

}  // namespace hydra
