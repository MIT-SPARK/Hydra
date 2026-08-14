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
#include "hydra_ros/places/gvd_places_visualizer.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/printing.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/gvd_place_extractor.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/drawing.h>

#include <rclcpp/time.hpp>

#include "hydra_ros/visualizer/voxel_drawing.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GvdPlaceExtractor::Sink,
                                   GvdPlacesVisualizer,
                                   GvdPlacesVisualizer::Config>("GvdPlacesVisualizer");

}

using places::GraphExtractor;
using places::GvdLayer;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using visualizer::RangeColormap;

void declare_config(GvdPlacesVisualizer::Config& config) {
  using namespace config;
  name("GvdPlacesVisualizer::Config");
  field(config.ns, "ns");
  field(config.gvd, "gvd");
  field(config.compressed_node_scale, "compressed_node_scale");
  field(config.compressed_edge_scale, "compressed_edge_scale");
  field(config.compressed_alpha, "compressed_alpha");
  field(config.colormap, "colormap");
  field(config.block_color, "block_color");

  check(config.compressed_node_scale, GT, 0.0, "compressed_node_scale");
  check(config.compressed_edge_scale, GT, 0.0, "compressed_edge_scale");
  checkInRange(config.compressed_alpha, 0.0, 1.0, "compressed_alpha", false, true);
}

GvdPlacesVisualizer::GvdPlacesVisualizer(const Config& config)
    : nh_(ianvs::NodeHandle::this_node(config.ns)),
      pubs_(nh_),
      config_("gvd_places_visualizer", config) {}

std::string GvdPlacesVisualizer::printInfo() const {
  return config::toString(config_.get());
}

void GvdPlacesVisualizer::call(uint64_t timestamp_ns,
                               const Eigen::Isometry3d& pose,
                               const GvdLayer& gvd,
                               const GraphExtractor& extractor) const {
  std_msgs::msg::Header header;
  header.frame_id = GlobalInfo::instance().getFrames().map;
  header.stamp = rclcpp::Time(timestamp_ns);

  const auto config = config_.get();
  const RangeColormap sdf_cmap(config.colormap);
  pubs_.publish("esdf_viz", header, [&]() -> Marker {
    return drawEsdf(config.gvd, sdf_cmap, pose, gvd, "esdf");
  });

  visualizeGvd(config, header, gvd);
  visualizeExtractor(config, header, extractor);
}

void GvdPlacesVisualizer::visualizeGvd(const Config& config,
                                       const std_msgs::msg::Header& header,
                                       const GvdLayer& gvd) const {
  const RangeColormap cmap(config.colormap);
  pubs_.publish("gvd_voxels", header, [&]() -> Marker {
    return drawGvd(config.gvd, cmap, gvd, "gvd_voxels");
  });

  pubs_.publish("surface_voxels", header, [&]() -> Marker {
    return drawGvdSurface(config.gvd, cmap, gvd, "surface_voxels");
  });

  ActiveBlockColoring block_cmap(config.block_color);
  pubs_.publish("active_gvd_blocks", header, [&]() -> Marker {
    return drawSpatialGrid(gvd,
                           config.gvd.block_outline_scale,
                           "active_gvd_blocks",
                           1.0,
                           block_cmap.getCallback<places::GvdBlock>());
  });
}

void GvdPlacesVisualizer::visualizeExtractor(const Config& config,
                                             const std_msgs::msg::Header& header,
                                             const GraphExtractor& extractor) const {
  const RangeColormap cmap(config.colormap);
  pubs_.publish("compressed_graph", header, [&]() -> MarkerArray {
    return drawCompressedGraph(extractor.graph(),
                               config.gvd,
                               cmap,
                               "compressed_graph",
                               config.compressed_node_scale,
                               config.compressed_edge_scale,
                               config.compressed_alpha);
  });

  pubs_.publish("gvd_wireframe", header, [&]() -> MarkerArray {
    return drawGvdWireframe(extractor.gvd(), config.gvd, cmap, "gvd_wireframe");
  });

  pubs_.publish("gvd_active", header, [&]() -> MarkerArray {
    return drawGvdActive(extractor.gvd(), config.gvd, "gvd_active");
  });

  pubs_.publish("gvd_active_cluster", header, [&]() -> MarkerArray {
    return drawGvdActiveCluster(extractor.gvd(), config.gvd, "gvd_active_cluster");
  });

  pubs_.publish("gvd_clusters", header, [&]() -> MarkerArray {
    return drawGvdClusters(extractor.gvd(), config.gvd, "gvd_clusters");
  });
}

}  // namespace hydra
