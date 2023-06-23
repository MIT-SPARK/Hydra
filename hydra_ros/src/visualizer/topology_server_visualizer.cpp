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
#include "hydra_ros/visualizer/topology_server_visualizer.h"

#include <hydra/utils/timing_utilities.h>

#include "hydra_ros/config/ros_utilities.h"

namespace hydra {

using places::CompressionGraphExtractor;
using places::GvdGraph;
using places::GvdVoxel;
using timing::ScopedTimer;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;
using voxblox::Layer;
using voxblox::MeshLayer;
using voxblox::TsdfVoxel;

TopologyServerVisualizer::TopologyServerVisualizer(const std::string& ns)
    : nh_(ns), previous_spheres_(0), published_gvd_graph_(false) {
  pubs_.reset(new MarkerGroupPub(nh_));

  config_ = config_parser::load_from_ros_nh<TopologyVisualizerConfig>(nh_);
  config_.graph.layer_z_step = 0;
  config_.graph.color_places_by_distance = true;

  setupConfigServers();
}

void TopologyServerVisualizer::visualize(const SceneGraphLayer& graph,
                                         const GvdGraph& gvd_graph,
                                         const Layer<GvdVoxel>& gvd,
                                         const Layer<TsdfVoxel>& tsdf,
                                         uint64_t timestamp_ns,
                                         const MeshLayer* mesh) {
  ScopedTimer timer("topology/topology_visualizer", timestamp_ns);

  std_msgs::Header header;
  header.frame_id = config_.world_frame;
  header.stamp.fromNSec(timestamp_ns);

  visualizeGraph(header, graph);
  visualizeGvdGraph(header, gvd_graph);
  visualizeGvd(header, gvd);
  if (config_.show_block_outlines) {
    visualizeBlocks(header, gvd, tsdf, mesh);
  }
}

void TopologyServerVisualizer::visualizeExtractor(
    uint64_t timestamp_ns, const CompressionGraphExtractor& extractor) {
  std_msgs::Header header;
  header.frame_id = config_.world_frame;
  header.stamp.fromNSec(timestamp_ns);

  MarkerArray markers;
  pubs_->publish("gvd_cluster_viz", [&](MarkerArray& markers) {
    const std::string ns = "gvd_cluster_graph";
    if (extractor.getGvdGraph().empty() && published_gvd_clusters_) {
      published_gvd_graph_ = false;
      markers.markers.push_back(makeDeleteMarker(header, 0, ns + "_nodes"));
      markers.markers.push_back(makeDeleteMarker(header, 0, ns + "_edges"));
      return true;
    }

    markers = showGvdClusters(extractor.getGvdGraph(),
                              extractor.getCompressedNodeInfo(),
                              extractor.getCompressedRemapping(),
                              config_.gvd,
                              config_.colormap,
                              ns);

    if (markers.markers.empty()) {
      return false;
    }

    markers.markers.at(0).header = header;
    markers.markers.at(1).header = header;
    published_gvd_clusters_ = true;
    return true;
  });
}

void TopologyServerVisualizer::visualizeError(const Layer<GvdVoxel>& lhs,
                                              const Layer<GvdVoxel>& rhs,
                                              double threshold,
                                              uint64_t timestamp_ns) {
  pubs_->publish("error_viz", [&](Marker& msg) {
    msg = makeErrorMarker(config_.gvd, config_.colormap, lhs, rhs, threshold);
    msg.header.frame_id = config_.world_frame;
    msg.header.stamp.fromNSec(timestamp_ns);

    if (msg.points.size()) {
      return true;
    } else {
      LOG(INFO) << "no voxels with error above threshold";
      return false;
    }
  });
}

void TopologyServerVisualizer::visualizeGraph(const std_msgs::Header& header,
                                              const SceneGraphLayer& graph) {
  if (graph.nodes().empty()) {
    LOG(INFO) << "visualizing empty graph!";
    return;
  }

  pubs_->publish("graph_viz", [&](MarkerArray& markers) {
    const std::string node_ns = config_.topology_marker_ns + "_nodes";
    Marker node_marker = makeCentroidMarkers(
        header, config_.graph_layer, graph, config_.graph, node_ns, config_.colormap);
    markers.markers.push_back(node_marker);

    if (!graph.edges().empty()) {
      Marker edge_marker = makeLayerEdgeMarkers(header,
                                                config_.graph_layer,
                                                graph,
                                                config_.graph,
                                                NodeColor::Zero(),
                                                config_.topology_marker_ns + "_edges");
      markers.markers.push_back(edge_marker);
    }

    return true;
  });

  publishFreespace(header, graph);
  publishGraphLabels(header, graph);
}

void TopologyServerVisualizer::visualizeGvdGraph(const std_msgs::Header& header,
                                                 const GvdGraph& graph) const {
  pubs_->publish("gvd_graph_viz", [&](MarkerArray& markers) {
    const std::string ns = config_.topology_marker_ns + "_gvd_graph";
    if (graph.empty() && published_gvd_graph_) {
      published_gvd_graph_ = false;
      markers.markers.push_back(makeDeleteMarker(header, 0, ns + "_nodes"));
      markers.markers.push_back(makeDeleteMarker(header, 0, ns + "_edges"));
      return true;
    }

    markers = makeGvdGraphMarkers(graph, config_.gvd, config_.colormap, ns);
    if (markers.markers.empty()) {
      return false;
    }

    markers.markers.at(0).header = header;
    markers.markers.at(1).header = header;
    published_gvd_graph_ = true;
    return true;
  });
}

void TopologyServerVisualizer::visualizeGvd(const std_msgs::Header& header,
                                            const Layer<GvdVoxel>& gvd) const {
  pubs_->publish("esdf_viz", [&](Marker& msg) {
    msg = makeEsdfMarker(config_.gvd, config_.colormap, gvd);
    msg.header = header;
    msg.ns = "gvd_visualizer";

    if (msg.points.size()) {
      return true;
    } else {
      LOG(INFO) << "visualizing empty ESDF slice";
      return false;
    }
  });

  pubs_->publish("gvd_viz", [&](Marker& msg) {
    msg = makeGvdMarker(config_.gvd, config_.colormap, gvd);
    msg.header = header;
    msg.ns = "gvd_visualizer";

    if (msg.points.size()) {
      return true;
    } else {
      LOG(INFO) << "visualizing empty GVD slice";
      return false;
    }
  });

  pubs_->publish("surface_viz", [&](Marker& msg) {
    msg = makeSurfaceVoxelMarker(config_.gvd, config_.colormap, gvd);
    msg.header = header;
    msg.ns = "gvd_visualizer";

    if (msg.points.size()) {
      return true;
    } else {
      LOG(INFO) << "visualizing empty surface slice";
      return false;
    }
  });
}

void TopologyServerVisualizer::visualizeBlocks(const std_msgs::Header& header,
                                               const Layer<GvdVoxel>& gvd,
                                               const Layer<TsdfVoxel>& tsdf,
                                               const MeshLayer* mesh) const {
  pubs_->publish("voxel_block_viz", [&](Marker& msg) {
    if (config_.use_gvd_block_outlines) {
      msg = makeBlocksMarker(gvd, config_.outline_scale);
    } else {
      msg = makeBlocksMarker(tsdf, config_.outline_scale);
    }

    msg.header = header;
    msg.ns = "topology_server_blocks";
    return true;
  });

  if (mesh) {
    pubs_->publish("mesh_block_viz", [&](Marker& msg) {
      msg = makeMeshBlocksMarker(*mesh, config_.outline_scale);
      msg.header = header;
      msg.ns = "topology_server_mesh_blocks";
      return true;
    });
  }
}

void TopologyServerVisualizer::publishFreespace(const std_msgs::Header& header,
                                                const SceneGraphLayer& graph) {
  const std::string label_ns = config_.topology_marker_ns + "_freespace";

  MarkerArray spheres = makePlaceSpheres(header, graph, label_ns, 0.15);

  MarkerArray delete_markers;
  for (size_t id = 0; id < previous_spheres_; ++id) {
    Marker delete_label;
    delete_label.action = Marker::DELETE;
    delete_label.id = id;
    delete_label.ns = label_ns;
    delete_markers.markers.push_back(delete_label);
  }
  previous_spheres_ = spheres.markers.size();

  // there's not really a clean way to delay computation on either of these markers, so
  // we just assign the messages in the callbacks
  pubs_->publish("freespace_viz", [&](MarkerArray& msg) {
    msg = delete_markers;
    return true;
  });
  pubs_->publish("freespace_viz", [&](MarkerArray& msg) {
    msg = spheres;
    return true;
  });

  pubs_->publish("freespace_graph_viz", [&](MarkerArray& markers) {
    const std::string node_ns = config_.topology_marker_ns + "_freespace_nodes";
    auto freespace_conf = config_.graph_layer;
    freespace_conf.use_sphere_marker = false;
    freespace_conf.marker_scale = 0.08;
    freespace_conf.marker_alpha = 0.5;
    Marker node_marker = makeCentroidMarkers(
        header, freespace_conf, graph, config_.graph, node_ns, [](const auto&) {
          return NodeColor::Zero();
        });
    markers.markers.push_back(node_marker);

    if (!graph.edges().empty()) {
      Marker edge_marker =
          makeLayerEdgeMarkers(header,
                               config_.graph_layer,
                               graph,
                               config_.graph,
                               NodeColor::Zero(),
                               config_.topology_marker_ns + "_freespace_edges");
      markers.markers.push_back(edge_marker);
    }

    return true;
  });
}

void TopologyServerVisualizer::publishGraphLabels(const std_msgs::Header& header,
                                                  const SceneGraphLayer& graph) {
  if (!config_.graph_layer.use_label) {
    return;
  }

  const std::string label_ns = config_.topology_marker_ns + "_labels";

  MarkerArray labels;
  for (const auto& id_node_pair : graph.nodes()) {
    const SceneGraphNode& node = *id_node_pair.second;
    Marker label =
        makeTextMarker(header, config_.graph_layer, node, config_.graph, label_ns);
    labels.markers.push_back(label);
  }

  std::set<int> current_ids;
  for (const auto& label : labels.markers) {
    current_ids.insert(label.id);
  }

  std::set<int> ids_to_delete;
  for (auto previous : previous_labels_) {
    if (!current_ids.count(previous)) {
      ids_to_delete.insert(previous);
    }
  }
  previous_labels_ = current_ids;

  MarkerArray delete_markers;
  for (auto to_delete : ids_to_delete) {
    Marker delete_label;
    delete_label.action = Marker::DELETE;
    delete_label.id = to_delete;
    delete_label.ns = label_ns;
    delete_markers.markers.push_back(delete_label);
  }

  // there's not really a clean way to delay computation on either of these markers, so
  // we just assign the messages in the callbacks
  pubs_->publish("graph_label_viz", [&](MarkerArray& msg) {
    msg = delete_markers;
    return true;
  });
  pubs_->publish("graph_label_viz", [&](MarkerArray& msg) {
    msg = labels;
    return true;
  });
}

void TopologyServerVisualizer::graphConfigCb(LayerConfig& config, uint32_t) {
  config_.graph_layer = config;
}

void TopologyServerVisualizer::colormapCb(ColormapConfig& config, uint32_t) {
  config_.colormap = config;
}

void TopologyServerVisualizer::gvdConfigCb(GvdVisualizerConfig& config, uint32_t) {
  config_.gvd = config;
  config_.graph.places_colormap_min_distance = config.gvd_min_distance;
  config_.graph.places_colormap_max_distance = config.gvd_max_distance;
}

void TopologyServerVisualizer::setupConfigServers() {
  startRqtServer(
      "gvd_visualizer", gvd_config_server_, &TopologyServerVisualizer::gvdConfigCb);

  startRqtServer("graph_visualizer",
                 graph_config_server_,
                 &TopologyServerVisualizer::graphConfigCb);

  startRqtServer(
      "visualizer_colormap", colormap_server_, &TopologyServerVisualizer::colormapCb);
}

}  // namespace hydra
