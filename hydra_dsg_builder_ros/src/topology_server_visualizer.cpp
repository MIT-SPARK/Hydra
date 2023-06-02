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
#include "hydra_dsg_builder_ros/topology_server_visualizer.h"

#include <hydra_utils/timing_utilities.h>

namespace hydra {

using timing::ScopedTimer;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace {

inline Marker makeDeleteMarker(const std_msgs::Header& header,
                               size_t id,
                               const std::string& ns) {
  Marker marker;
  marker.header = header;
  marker.action = Marker::DELETE;
  marker.id = id;
  marker.ns = ns;
  return marker;
}

}  // namespace

TopologyServerVisualizer::TopologyServerVisualizer(const std::string& ns)
    : nh_(ns), previous_spheres_(0), published_gvd_graph_(false) {
  pubs_.reset(new MarkerGroupPub(nh_));

  config_ = config_parser::load_from_ros_nh<TopologyVisualizerConfig>(nh_);
  startRqtServer(
      "gvd_visualizer", gvd_config_server_, &TopologyServerVisualizer::gvdConfigCb);
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
    uint64_t timestamp_ns,
    const CompressionGraphExtractor& extractor) {
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
    ROS_ERROR("Not implemented");
    return false;
  });

  publishFreespace(header, graph);
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
    ROS_ERROR("Not implemented yet");
    return false;
  });
}

void TopologyServerVisualizer::gvdConfigCb(GvdVisualizerConfig& config, uint32_t) {
  config_.gvd = config;
}

}  // namespace hydra
