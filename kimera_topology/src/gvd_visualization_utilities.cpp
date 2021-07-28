#include "kimera_topology/gvd_visualization_utilities.h"

#include <tf2_eigen/tf2_eigen.h>

#include <random>

using visualization_msgs::Marker;

namespace kimera {
namespace topology {

#define RETURN_MODE_IF_MATCH(mode, string) \
  if (string == #mode) {                   \
    return GvdVisualizationMode::mode;     \
  }                                        \
  static_assert(true, "")

GvdVisualizationMode getModeFromString(const std::string& mode) {
  RETURN_MODE_IF_MATCH(DEFAULT, mode);
  RETURN_MODE_IF_MATCH(DISTANCE, mode);
  RETURN_MODE_IF_MATCH(BASIS_POINTS, mode);
  ROS_WARN_STREAM("Invalid mode: " << mode << ". Using DEFAULT color mode");
  return GvdVisualizationMode::DEFAULT;
}

#undef RETURN_MODE_IF_MATCH

double computeRatio(double min, double max, double value) {
  double ratio = (value - min) / (max - min);
  ratio = !std::isfinite(ratio) ? 0.0 : ratio;
  return ratio;
}

double getRatioFromDistance(const GvdVisualizationConfig& config,
                            const GvdVoxel& voxel) {
  return computeRatio(config.min_distance, config.max_distance, voxel.distance);
}

double getRatioFromBasisPoints(const GvdVisualizationConfig& config,
                               const GvdVoxel& voxel) {
  return computeRatio(static_cast<double>(config.min_num_basis),
                      static_cast<double>(config.max_num_basis),
                      static_cast<double>(voxel.num_extra_basis));
}

double getRatio(const GvdVisualizationConfig& config, const GvdVoxel& voxel) {
  switch (config.mode) {
    case GvdVisualizationMode::BASIS_POINTS:
      return getRatioFromBasisPoints(config, voxel);
    case GvdVisualizationMode::DISTANCE:
    case GvdVisualizationMode::DEFAULT:
    default:
      return getRatioFromDistance(config, voxel);
  }
  return 0.0;
}

Marker makeGvdMarker(const Layer<GvdVoxel>& layer,
                     const dsg_utils::HlsColorMapConfig& color_config,
                     const GvdVisualizationConfig& gvd_config) {
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "gvd_markers";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size();
  marker.scale.y = layer.voxel_size();
  marker.scale.z = layer.voxel_size();

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.observed || voxel.num_extra_basis < gvd_config.basis_threshold) {
        continue;
      }

      Eigen::Vector3d voxel_pos =
          block.computeCoordinatesFromLinearIndex(i).cast<double>();
      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      double ratio = getRatio(gvd_config, voxel);
      NodeColor color = dsg_utils::interpolateColorMap(color_config, ratio);

      std_msgs::ColorRGBA color_msg = dsg_utils::makeColorMsg(color, gvd_config.alpha);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

Marker makeEsdfMarker(const Layer<GvdVoxel>& layer,
                      const dsg_utils::HlsColorMapConfig& color_config,
                      const EsdfVisualizationConfig& esdf_config) {
  BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);

  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "esdf_slice_markers";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size();
  marker.scale.y = layer.voxel_size();
  marker.scale.z = layer.voxel_size();

  const FloatingPoint voxel_size = layer.voxel_size();
  const FloatingPoint half_voxel_size = voxel_size / 2.0;
  // rounds down and points the slice at the middle of the nearest voxel boundary
  const FloatingPoint slice_height =
      std::floor(esdf_config.slice_height / voxel_size) * voxel_size + half_voxel_size;

  for (const auto& idx : blocks) {
    const auto& block = layer.getBlockByIndex(idx);
    for (size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!voxel.observed) {
        continue;
      }

      Eigen::Vector3d voxel_pos =
          block.computeCoordinatesFromLinearIndex(i).cast<double>();
      if (voxel_pos(2) < slice_height - half_voxel_size ||
          voxel_pos(2) > slice_height + half_voxel_size) {
        continue;
      }

      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      double ratio = computeRatio(
          esdf_config.min_distance, esdf_config.max_distance, voxel.distance);
      NodeColor color = dsg_utils::interpolateColorMap(color_config, ratio);

      std_msgs::ColorRGBA color_msg = dsg_utils::makeColorMsg(color, esdf_config.alpha);
      marker.colors.push_back(color_msg);
    }
  }

  return marker;
}

Marker makeGvdEdgeMarker(const Layer<GvdVoxel>& layer,
                         const EdgeInfoMap& edge_info_map,
                         const IdIndexMap& id_root_index_map) {
  const double alpha = 0.8;
  Marker marker;
  marker.type = Marker::CUBE_LIST;
  marker.action = Marker::ADD;
  marker.id = 0;
  marker.ns = "gvd_edge_markers";

  Eigen::Vector3d identity_pos = Eigen::Vector3d::Zero();
  tf2::convert(identity_pos, marker.pose.position);
  tf2::convert(Eigen::Quaterniond::Identity(), marker.pose.orientation);

  marker.scale.x = layer.voxel_size();
  marker.scale.y = layer.voxel_size();
  marker.scale.z = layer.voxel_size();

  for (const auto& id_index_pair : id_root_index_map) {
    Eigen::Vector3d voxel_pos = getVoxelPosition(layer, id_index_pair.second);

    geometry_msgs::Point marker_pos;
    tf2::convert(voxel_pos, marker_pos);
    marker.points.push_back(marker_pos);
    marker.colors.push_back(dsg_utils::makeColorMsg(NodeColor::Zero(), alpha));
  }

  size_t next_color_id = 0;
  std::map<size_t, size_t> edge_color_map;
  for (const auto& id_edge_pair : edge_info_map) {
    std::set<size_t> neighbor_colors;
    for (auto other_edge : id_edge_pair.second.connections) {
      if (!edge_color_map.count(other_edge)) {
        continue;
      }

      neighbor_colors.insert(edge_color_map.at(other_edge));
    }

    bool found_color = false;
    for (size_t i = 0; i < next_color_id; ++i) {
      if (neighbor_colors.count(i)) {
        continue;
      }

      found_color = true;
      edge_color_map[id_edge_pair.first] = i;
      break;
    }

    if (found_color) {
      continue;
    }

    edge_color_map[id_edge_pair.first] = next_color_id;
    next_color_id++;
  }

  std::vector<double> hues;
  for (size_t i = 0; i < next_color_id; ++i) {
    hues.push_back(static_cast<double>(i) / static_cast<double>(next_color_id));
  }
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(hues.begin(), hues.end(), g);

  for (const auto& id_edge_pair : edge_info_map) {
    cv::Mat hls_value(1, 1, CV_32FC3);
    hls_value.at<float>(0) = hues.at(edge_color_map.at(id_edge_pair.first)) * 360.0;
    hls_value.at<float>(1) = 0.7;
    hls_value.at<float>(2) = 0.9;

    cv::Mat bgr;
    cv::cvtColor(hls_value, bgr, cv::COLOR_HLS2BGR);
    NodeColor color;
    color(0, 0) = static_cast<uint8_t>(255 * bgr.at<float>(2));
    color(1, 0) = static_cast<uint8_t>(255 * bgr.at<float>(1));
    color(2, 0) = static_cast<uint8_t>(255 * bgr.at<float>(0));

    for (const auto& index : id_edge_pair.second.indices) {
      Eigen::Vector3d voxel_pos = getVoxelPosition(layer, index);

      geometry_msgs::Point marker_pos;
      tf2::convert(voxel_pos, marker_pos);
      marker.points.push_back(marker_pos);

      // TODO(nathan) get color
      marker.colors.push_back(dsg_utils::makeColorMsg(color, alpha));
    }
  }

  return marker;
}

}  // namespace topology
}  // namespace kimera
