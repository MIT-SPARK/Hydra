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
#include "hydra/frontend/place_2d_segmenter.h"

#include <glog/logging.h>
#include <kimera_semantics/color.h>
#include <pcl/segmentation/extract_clusters.h>
#include <spark_dsg/bounding_box_extraction.h>

#include "hydra/common/hydra_config.h"

namespace hydra {

using Places = Place2dSegmenter::Places;
using LabelPlaces = Place2dSegmenter::LabelPlaces;
using LabelIndices = Place2dSegmenter::LabelIndices;
using IndicesVector = Place2dSegmenter::IndicesVector;
using OptPosition = std::optional<Eigen::Vector3d>;

void mergeList(std::vector<size_t>& lhs, const std::vector<int>& rhs) {
  std::unordered_set<size_t> seen(lhs.begin(), lhs.end());
  for (const auto idx : rhs) {
    if (seen.count(idx)) {
      continue;
    }

    lhs.push_back(idx);
    seen.insert(idx);
  }
}

template <typename T>
std::string printLabels(const std::set<T>& labels) {
  std::stringstream ss;
  ss << "[";
  auto iter = labels.begin();
  while (iter != labels.end()) {
    ss << static_cast<uint64_t>(*iter);
    ++iter;
    if (iter != labels.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

Place2dSegmenter::Place2dSegmenter(
    const Place2dSegmenterConfig& config,
    const MeshVertexCloud::Ptr& vertices,
    const std::shared_ptr<std::vector<uint32_t>>& mesh_labels)
    : full_mesh_vertices_(vertices),
      full_mesh_labels_(mesh_labels),
      config_(config),
      next_node_id_(config.prefix, 0) {
  VLOG(1) << "[Hydra Frontend] Detecting 2d places: " << printLabels(config.labels);
  for (const auto& label : config.labels) {
    active_places_[label] = std::set<NodeId>();
  }
}

using KdTreeT = pcl::search::KdTree<pcl::PointXYZRGBA>;

Places Place2dSegmenter::findPlaces(const MeshVertexCloud::Ptr& cloud,
                                    const pcl::IndicesPtr& cloud_indices) const {
  KdTreeT::Ptr tree(new KdTreeT());
  tree->setInputCloud(cloud, cloud_indices);

  pcl::EuclideanClusterExtraction<Place2d::PointT> estimator;
  estimator.setClusterTolerance(config_.cluster_tolerance);
  estimator.setMinClusterSize(config_.min_cluster_size);
  estimator.setMaxClusterSize(config_.max_cluster_size);
  estimator.setSearchMethod(tree);
  estimator.setInputCloud(cloud);
  estimator.setIndices(cloud_indices);

  std::vector<pcl::PointIndices> cluster_indices;
  estimator.extract(cluster_indices);

  Places places;
  places.resize(cluster_indices.size());
  for (size_t k = 0; k < places.size(); ++k) {
    places.at(k).indices = cluster_indices.at(k);
  }

  return places;
}

pcl::IndicesPtr getActivePlaceIndices(
    const pcl::IndicesPtr& indices,
    const std::map<uint32_t, std::set<NodeId>>& active_places,
    const kimera_pgmo::MeshDelta& delta,
    const DynamicSceneGraph& graph,
    const OptPosition& pos,
    const pcl::PointCloud<pcl::PointXYZRGBA>& mesh,
    double mesh_active_horizon,
    double place_active_horizon) {
  pcl::IndicesPtr active_indices;
  active_indices.reset(new IndicesVector());
  active_indices->reserve(indices->size());

  if (!pos) {
    active_indices = indices;
    VLOG(1) << "[Places 2d Segmenter] Active mesh indices: " << indices->size()
            << " (used: " << active_indices->size() << ")";
    return active_indices;
  }

  VLOG(1) << "[Places 2d Segmenter] n original active indices: " << indices->size();
  const Eigen::Vector3d root_pos = *pos;
  std::unordered_set<size_t> frozen_indices;
  for (auto kv : active_places) {
    std::set<NodeId> nodes = kv.second;
    for (NodeId nid : nodes) {
      auto& attrs = graph.getNode(nid)->get().attributes<Place2dNodeAttributes>();
      if ((attrs.position - root_pos).norm() > place_active_horizon) {
        auto iter = attrs.pcl_mesh_connections.begin();
        while (iter != attrs.pcl_mesh_connections.end()) {
          if (delta.deleted_indices.count(*iter)) {
            iter = attrs.pcl_mesh_connections.erase(iter);
            continue;
          }

          auto map_iter = delta.prev_to_curr.find(*iter);
          if (map_iter != delta.prev_to_curr.end()) {
            *iter = map_iter->second;
          }
          frozen_indices.insert(*iter);
          ++iter;
        }
      }
    }
  }

  VLOG(1) << "[Places 2d Segmenter] n frozen indices: " << frozen_indices.size();

  for (const size_t idx : *indices) {
    const auto& p = mesh.at(idx);
    const Eigen::Vector3d vertex_pos(p.x, p.y, p.z);
    if ((vertex_pos - root_pos).norm() < mesh_active_horizon &&
        !frozen_indices.count(idx)) {
      active_indices->push_back(idx);
    }
  }
  VLOG(1) << "[Places 2d Segmenter] n final active indices: " << active_indices->size();

  VLOG(1) << "[Places 2d Segmenter] Active mesh indices: " << indices->size()
          << " (used: " << active_indices->size() << ")";
  return active_indices;
}

std::vector<Place2d> decomposePlaces(const Place2d::CloudT::Ptr cloud,
                                     const std::vector<Place2d>& places,
                                     double min_size,
                                     size_t min_points) {
  std::vector<Place2d> final_places;
  for (auto p : places) {
    // Turn the point cloud points in this place into cv::Point2f
    std::vector<cv::Point2f> cv_points;
    std::vector<size_t> local_to_global_index;
    std::vector<size_t> place_indices(p.indices.indices.size());
    std::iota(std::begin(place_indices), std::end(place_indices), 0);
    for (int ix : p.indices.indices) {
      local_to_global_index.push_back(ix);
      cv::Point2f xy = {cloud->points[ix].x, cloud->points[ix].y};
      cv_points.push_back(xy);
    }

    // Recursively decompose initial place into smaller places
    std::vector<std::vector<size_t>> sub_places =
        decomposePlace(cv_points, place_indices, min_size, min_points);

    for (std::vector<size_t> sp : sub_places) {
      // save information about each final place
      std::vector<cv::Point2f> region_pts;
      std::vector<size_t> region_to_local;
      Place2d sub_place_2d;
      for (size_t i : sp) {
        region_to_local.push_back(i);
        region_pts.push_back(cv_points.at(i));
        sub_place_2d.indices.indices.push_back(local_to_global_index.at(i));
        const auto& cp = cloud->at(local_to_global_index.at(i));
        sub_place_2d.centroid.add(pcl::PointXYZ(cp.x, cp.y, cp.z));
      }

      // compute convex hull for each place
      std::vector<int> ch;
      cv::convexHull(region_pts, ch);
      for (int pix : ch) {
        size_t cloud_ix = local_to_global_index.at(region_to_local.at(pix));
        Place2d::PointT p = cloud->at(cloud_ix);
        Eigen::Vector3d v = {p.x, p.y, p.z};
        sub_place_2d.boundary.push_back(v);
        sub_place_2d.boundary_indices.indices.push_back(cloud_ix);
      }
      final_places.push_back(sub_place_2d);
    }
  }
  return final_places;
}

std::vector<std::vector<size_t>> decomposePlace(
    const std::vector<cv::Point2f>& cloud_pts,
    const std::vector<size_t>& indices,
    const double min_size,
    const size_t min_points) {
  // Base case: if there are too few points, return region
  if (indices.size() < min_points) {
    std::vector<std::vector<size_t>> indices_wrapped = {indices};
    return indices_wrapped;
  }

  // Compute min area rectangle to get general "shape" of points
  std::vector<cv::Point2f> region;
  for (size_t i : indices) {
    region.push_back(cloud_pts.at(i));
  }

  cv::RotatedRect box = cv::minAreaRect(region);

  cv::Point2f box_pts[4];
  box.points(box_pts);

  // Get rays along two sides of bounding box
  cv::Point2f base_pt = box_pts[0];
  cv::Point2f e1_ray = box_pts[1] - box_pts[0];
  cv::Point2f e2_ray = box_pts[3] - box_pts[0];

  cv::Point2f long_ray;
  cv::Point2f short_ray;
  if (cv::norm(e1_ray) > cv::norm(e2_ray)) {
    long_ray = e1_ray;
    short_ray = e2_ray;
  } else {
    long_ray = e2_ray;
    short_ray = e1_ray;
  }

  if (cv::norm(long_ray) > min_size) {
    std::vector<size_t> indices_1;
    std::vector<size_t> indices_2;

    // Split points perpendicular to long axis of bounding box
    cv::Point2f cut_origin = base_pt + long_ray / 2 - short_ray;
    for (size_t ix = 0; ix < region.size(); ++ix) {
      cv::Point2f pt = region[ix];
      double side = (pt - cut_origin).dot(long_ray);
      if (side >= 0) {
        indices_1.push_back(indices[ix]);
      }
      if (side <= 0) {
        indices_2.push_back(indices[ix]);
      }
    }

    std::vector<std::vector<size_t>> descendants1 =
        decomposePlace(cloud_pts, indices_1, min_size, min_points);
    std::vector<std::vector<size_t>> descendants2 =
        decomposePlace(cloud_pts, indices_2, min_size, min_points);
    descendants1.insert(descendants1.end(), descendants2.begin(), descendants2.end());
    return descendants1;
  } else {
    // Base Case: if region is small enough, don't split further
    std::vector<std::vector<size_t>> indices_wrapped = {indices};
    return indices_wrapped;
  }
}

NodeIdSet Place2dSegmenter::getActiveNodes() const {
  std::unordered_set<NodeId> all_active_nodes;
  for (auto kv : active_places_) {
    all_active_nodes.insert(kv.second.begin(), kv.second.end());
  }
  return all_active_nodes;
}

void Place2dSegmenter::detect(const ReconstructionOutput& msg,
                              const kimera_pgmo::MeshDelta::Ptr& mesh_delta,
                              const DynamicSceneGraph& graph) {
  std::optional<Eigen::Vector3d> pos = msg.current_position;

  VLOG(1) << "[Places 2d Segmenter] detect called";
  const auto active_indices = getActivePlaceIndices(mesh_delta->getActiveIndices(),
                                                    active_places_,
                                                    *mesh_delta,
                                                    graph,
                                                    pos,
                                                    *full_mesh_vertices_,
                                                    config_.mesh_active_window_m,
                                                    config_.active_place_radius_m);

  LabelPlaces label_places;

  if (active_indices->empty()) {
    VLOG(1) << "[Places 2d Segmenter] No active indices in mesh";
    detected_label_places_ = label_places;
    return;
  }

  LabelIndices label_indices = getLabelIndices(*active_indices);
  if (label_indices.empty()) {
    VLOG(1) << "[Places 2d Segmenter] No vertices found matching desired labels";
    for (const auto& callback_func : callback_funcs_) {
      callback_func(*full_mesh_vertices_, *active_indices, label_indices);
    }
    detected_label_places_ = label_places;
    return;
  }

  for (const auto label : config_.labels) {
    if (!label_indices.count(label)) {
      continue;
    }

    if (label_indices.at(label)->size() < config_.min_cluster_size) {
      continue;
    }

    const auto initial_places =
        findPlaces(full_mesh_vertices_, label_indices.at(label));

    VLOG(1) << "[Places 2d Segmenter] got " << initial_places.size()
            << " initial places";
    std::vector<Place2d> final_places = decomposePlaces(full_mesh_vertices_,
                                                        initial_places,
                                                        config_.min_final_place_size,
                                                        config_.min_final_place_points);

    VLOG(1) << "[Places 2d Segmenter]  - Found " << final_places.size()
            << " final places of label " << static_cast<int>(label);
    label_places.insert({label, final_places});
  }

  for (const auto& callback_func : callback_funcs_) {
    callback_func(*full_mesh_vertices_, *active_indices, label_indices);
  }

  detected_label_places_ = label_places;
}

LabelIndices Place2dSegmenter::getLabelIndices(const IndicesVector& indices) const {
  LabelIndices label_indices;

  std::set<uint32_t> seen_labels;
  for (const auto idx : indices) {
    if (static_cast<size_t>(idx) >= full_mesh_labels_->size()) {
      LOG(ERROR) << "bad index " << idx << "(of " << full_mesh_labels_->size() << ")";
      continue;
    }

    const auto label = full_mesh_labels_->at(idx);
    seen_labels.insert(label);

    if (!config_.labels.count(label)) {
      continue;
    }

    if (!label_indices.count(label)) {
      label_indices[label] = pcl::IndicesPtr(new IndicesVector());
    }

    label_indices[label]->push_back(idx);
  }

  VLOG(3) << "[Places 2d Segmenter] Seen labels: " << printLabels(seen_labels);

  return label_indices;
}

void Place2dSegmenter::updateGraph(uint64_t timestamp_ns,
                                   const ReconstructionOutput& msg,
                                   DynamicSceneGraph& graph) {
  std::optional<Eigen::Vector3d> pos = msg.current_position;
  VLOG(1) << "[Places 2d Segmenter] updateGraph";
  std::map<uint32_t, std::set<NodeId>> new_active_places;
  for (const auto& label : config_.labels) {
    new_active_places[label] = std::set<NodeId>();
  }

  if (!pos) {
    new_active_places = active_places_;
  } else {
    int frozen_but_kept = 0;
    int cleaned = 0;
    int forgotten = 0;
    for (auto kv : active_places_) {
      std::set<NodeId> nodes = kv.second;
      for (NodeId nid : nodes) {
        const SceneGraphNode& n = graph.getNode(nid).value();
        double r = (n.attributes().position - *pos).norm();
        double upper = config_.place_memory_radius_m;
        double lower = config_.active_place_radius_m;
        if (r >= lower && r <= upper) {
          // keep track of places that are close enough to affect future iterations
          new_active_places.at(kv.first).insert(nid);
          frozen_but_kept++;
        } else if (r < lower) {
          graph.removeNode(nid);
          cleaned++;
        } else {
          graph.getNode(nid)->get().attributes().is_active = false;
          forgotten++;
        }
      }
    }

    VLOG(1) << "[Places 2d Segmenter] " << frozen_but_kept << " places frozen but kept";
    VLOG(1) << "[Places 2d Segmenter] " << cleaned << " places cleaned";
    VLOG(1) << "[Places 2d Segmenter] " << forgotten << " places forgotten";
  }

  for (const auto& label_places : detected_label_places_) {
    for (const auto& place : label_places.second) {
      NodeSymbol ns = addPlaceToGraph(graph, place, label_places.first, timestamp_ns);
      new_active_places.at(label_places.first).insert(ns);
    }
  }

  std::set<NodeId> full_nodes;
  for (auto kv : new_active_places) {
    std::set<NodeId> nodes = kv.second;
    for (NodeId nid : nodes) {
      full_nodes.insert(nid);
    }
  }

  for (NodeSymbol ns1 : full_nodes) {
    auto c1 = graph.getNode(ns1)->get().attributes().position;
    for (NodeSymbol ns2 : full_nodes) {
      if (ns1 == ns2) {
        continue;
      }
      auto c2 = graph.getNode(ns2)->get().attributes().position;
      if ((c1 - c2).norm() < (config_.min_final_place_size * 1.3)) {
        graph.insertEdge(ns1, ns2);
      }
    }
  }

  active_places_ = new_active_places;
}

NodeSymbol Place2dSegmenter::addPlaceToGraph(DynamicSceneGraph& graph,
                                             const Place2d& place,
                                             uint32_t label,
                                             uint64_t timestamp) {
  if (place.indices.indices.size() == 0) {
    LOG(ERROR) << "Encountered empty place with label" << static_cast<int>(label)
               << " @ " << timestamp << "[ns]";
    return next_node_id_;  // TODO(aaron): probably shouldn't do this, but it works for
                           // now
  }

  Place2dNodeAttributes::Ptr attrs = std::make_unique<Place2dNodeAttributes>();
  attrs->semantic_label = label;
  attrs->name = NodeSymbol(next_node_id_).getLabel();
  attrs->boundary = place.boundary;
  attrs->pcl_boundary_connections.insert(attrs->pcl_boundary_connections.begin(),
                                         place.boundary_indices.indices.begin(),
                                         place.boundary_indices.indices.end());
  // TODO(aaron): figure out bounding box
  // attrs->bounding_box = bounding_box::extract(
  //    place.cloud, config_.bounding_box_type, nullptr, config_.angle_step);

  const auto& label_to_name = HydraConfig::instance().getLabelToNameMap();
  auto iter = label_to_name.find(label);
  if (iter != label_to_name.end()) {
    attrs->name = iter->second;
  } else {
    VLOG(1) << "Missing semantic label from map: " << std::to_string(label);
  }

  attrs->pcl_mesh_connections.insert(attrs->pcl_mesh_connections.begin(),
                                     place.indices.indices.begin(),
                                     place.indices.indices.end());

  auto label_map = HydraConfig::instance().getSemanticColorMap();
  if (!label_map || !label_map->isValid()) {
    label_map = HydraConfig::instance().setRandomColormap();
    CHECK(label_map != nullptr);
  }

  const auto color = label_map->getColorFromLabel(label);
  attrs->color << color.r, color.g, color.b;

  pcl::PointXYZ centroid;
  place.centroid.get(centroid);
  attrs->position << centroid.x, centroid.y, centroid.z;
  attrs->is_active = true;

  graph.emplaceNode(DsgLayers::MESH_PLACES, next_node_id_, std::move(attrs));

  active_places_.at(label).insert(next_node_id_);
  active_place_timestamps_[next_node_id_] = timestamp;

  return next_node_id_++;
}

}  // namespace hydra
