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
#include "hydra/frontend/place_2d_ellipsoid_math.h"

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
      next_node_id_(config.prefix, 0),
      num_archived_vertices_(0) {
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
    addPlaceRectInfo(cloud->points, places.at(k));
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
    size_t& num_archived_vertices,
    std::list<NodeId>& empty_nodes) {
  pcl::IndicesPtr active_indices;
  active_indices.reset(new IndicesVector());
  active_indices->reserve(indices->size());

  num_archived_vertices = delta.getTotalArchivedVertices();
  if (!pos) {
    active_indices = indices;
    VLOG(1) << "[Places 2d Segmenter] Active mesh indices: " << indices->size()
            << " (used: " << active_indices->size() << ")";
    return active_indices;
  }

  VLOG(1) << "[Places 2d Segmenter] n original active indices: " << indices->size();
  std::unordered_set<size_t> frozen_indices;
  for (auto kv : active_places) {
    std::set<NodeId> nodes = kv.second;
    for (NodeId nid : nodes) {
      auto& attrs = graph.getNode(nid)->get().attributes<Place2dNodeAttributes>();
      size_t min_index = SIZE_MAX;
      size_t max_index = 0;
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
        min_index = std::min(min_index, *iter);
        max_index = std::max(max_index, *iter);
        ++iter;
      }
      attrs.pcl_min_index = min_index;
      attrs.pcl_max_index = max_index;

      const auto prev_boundary = attrs.boundary;
      const auto prev_boundary_connections = attrs.pcl_boundary_connections;
      attrs.boundary.clear();
      attrs.pcl_boundary_connections.clear();
      for (size_t i = 0; i < prev_boundary.size(); ++i) {
        if (delta.deleted_indices.count(prev_boundary_connections.at(i))) {
          continue;
        }

        auto map_iter = delta.prev_to_curr.find(prev_boundary_connections.at(i));
        if (map_iter != delta.prev_to_curr.end()) {
          attrs.boundary.push_back(prev_boundary.at(i));
          attrs.pcl_boundary_connections.push_back(map_iter->second);
        } else {
          attrs.boundary.push_back(prev_boundary.at(i));
          attrs.pcl_boundary_connections.push_back(prev_boundary_connections.at(i));
        }
      }

      if (attrs.pcl_min_index < num_archived_vertices) {
        // ^ this means that the place contains an archived vertex
        for (size_t i : attrs.pcl_mesh_connections) {
          frozen_indices.insert(i);
        }
      }
      if (!attrs.is_active) {
        if (attrs.pcl_mesh_connections.size() == 0 || attrs.boundary.size() < 3) {
          empty_nodes.push_back(nid);
        }
      }
    }
  }

  VLOG(1) << "[Places 2d Segmenter] n frozen indices: " << frozen_indices.size();

  for (const size_t idx : *indices) {
    const auto& p = mesh.at(idx);
    const Eigen::Vector3d vertex_pos(p.x, p.y, p.z);
    if (!frozen_indices.count(idx)) {
      active_indices->push_back(idx);
    }
  }
  VLOG(1) << "[Places 2d Segmenter] n final active indices: " << active_indices->size();

  VLOG(1) << "[Places 2d Segmenter] Active mesh indices: " << indices->size()
          << " (used: " << active_indices->size() << ")";
  return active_indices;
}

std::vector<Place2d> decomposePlaces(const Place2d::CloudT::Ptr cloud,
                                     const std::vector<Place2d>& initial_places,
                                     double min_size,
                                     size_t min_points) {
  std::vector<Place2d> final_places;
  for (auto p : initial_places) {
    // Recursively decompose initial place into smaller places
    std::vector<Place2d> sub_places =
        decomposePlace(cloud->points, p, min_size, min_points);

    for (Place2d sp : sub_places) {
      addPlaceBoundaryInfo(cloud->points, sp);
      final_places.push_back(sp);
    }
  }

  return final_places;
}

void addPlaceBoundaryInfo(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        points,
    Place2d& place) {
  std::vector<cv::Point2f> region_pts;
  std::vector<size_t> region_to_cloud_index;
  for (size_t i : place.indices.indices) {
    region_to_cloud_index.push_back(i);
    region_pts.push_back(cv::Point2f(points[i].x, points[i].y));
    place.centroid.add(pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
  }

  // compute convex hull for each place
  std::vector<int> ch;
  cv::convexHull(region_pts, ch);
  for (int pix : ch) {
    size_t cloud_ix = region_to_cloud_index.at(pix);
    Place2d::PointT p = points[cloud_ix];
    Eigen::Vector3d v = {p.x, p.y, p.z};
    place.boundary.push_back(v);
    place.boundary_indices.indices.push_back(cloud_ix);
  }
}

void addPlaceRectInfo(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        points,
    Place2d& place) {
  std::vector<cv::Point2f> region;
  for (size_t i : place.indices.indices) {
    region.push_back(cv::Point2f(points[i].x, points[i].y));
  }

  cv::RotatedRect box = cv::minAreaRect(region);

  cv::Point2f box_pts[4];
  box.points(box_pts);

  // Get rays along two sides of bounding box
  cv::Point2f e1_ray = box_pts[1] - box_pts[0];
  cv::Point2f e2_ray = box_pts[3] - box_pts[0];

  cv::Point2f long_ray = cv::norm(e1_ray) > cv::norm(e2_ray) ? e1_ray : e2_ray;

  cv::Point2f box_center = (box_pts[0] + box_pts[1] + box_pts[2] + box_pts[3]) / 4;
  place.ellipse_centroid(0) = box_center.x;
  place.ellipse_centroid(1) = box_center.y;

  Eigen::Matrix<float, 2, 2> m;
  m(0, 0) = sqrt(2) *
            (e1_ray.x / 2);  // TODO(aaron): make sqrt(2) user-defined scaling parameter
  m(1, 0) = sqrt(2) * (e1_ray.y / 2);
  m(0, 1) = sqrt(2) * (e2_ray.x / 2);
  m(1, 1) = sqrt(2) * (e2_ray.y / 2);

  place.ellipse_matrix_expand = m;
  Eigen::Matrix<float, 2, 2> minv = m.inverse();
  place.ellipse_matrix_compress = minv.transpose() * minv;

  place.cut_plane(0) = long_ray.x;
  place.cut_plane(1) = long_ray.y;
}

std::pair<Place2d, Place2d> splitPlace(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        points,
    const Place2d& place) {
  Place2d new_place_1;
  Place2d new_place_2;

  size_t min_ix_1 = SIZE_MAX;
  size_t max_ix_1 = 0;
  size_t min_ix_2 = SIZE_MAX;
  size_t max_ix_2 = 0;

  for (size_t i : place.indices.indices) {
    Eigen::Vector2d pt(points[i].x, points[i].y);
    double side = (pt - place.ellipse_centroid).dot(place.cut_plane);
    if (side >= 0) {
      new_place_1.indices.indices.push_back(i);
      min_ix_1 = std::min(min_ix_1, i);
      max_ix_1 = std::max(max_ix_1, i);
    }
    if (side <= 0) {
      new_place_2.indices.indices.push_back(i);
      min_ix_2 = std::min(min_ix_2, i);
      max_ix_2 = std::max(max_ix_2, i);
    }
  }

  new_place_1.min_mesh_index = min_ix_1;
  new_place_1.max_mesh_index = max_ix_1;

  new_place_2.min_mesh_index = min_ix_2;
  new_place_2.max_mesh_index = max_ix_2;

  addPlaceRectInfo(points, new_place_1);
  addPlaceRectInfo(points, new_place_2);

  return std::pair(new_place_1, new_place_2);
}

std::vector<Place2d> decomposePlace(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        cloud_pts,
    const Place2d& place,
    const double min_size,
    const size_t min_points) {
  std::pair<Place2d, Place2d> children = splitPlace(cloud_pts, place);

  std::vector<Place2d> descendants;
  if (children.first.indices.indices.size() > min_points &&
      children.first.cut_plane.norm() > min_size) {
    descendants = decomposePlace(cloud_pts, children.first, min_size, min_points);
  } else {
    descendants.push_back(children.first);
  }

  if (children.second.indices.indices.size() > min_points &&
      children.second.cut_plane.norm() > min_size) {
    std::vector<Place2d> temp =
        decomposePlace(cloud_pts, children.second, min_size, min_points);
    descendants.insert(descendants.end(), temp.begin(), temp.end());
  } else {
    descendants.push_back(children.second);
  }

  return descendants;
}

NodeIdSet Place2dSegmenter::getActiveNodes() const {
  std::unordered_set<NodeId> all_active_nodes;
  for (auto kv : active_places_) {
    all_active_nodes.insert(kv.second.begin(), kv.second.end());
  }
  return all_active_nodes;
}

bool canSplit(const Place2d& p, const Place2dSegmenterConfig& cfg) {
  if (p.indices.indices.size() > cfg.min_final_place_points &&
      p.cut_plane.norm() > cfg.impure_final_place_size) {
    return true;
  } else {
    return false;
  }
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
                                                    num_archived_vertices_,
                                                    nodes_to_remove_);

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
                                                        config_.pure_final_place_size,
                                                        config_.min_final_place_points);

    VLOG(1) << "[Places 2d Segmenter]  - Found " << final_places.size()
            << " final places of label " << static_cast<int>(label);
    label_places.insert({label, final_places});
  }

  if (config_.enable_place_purity) {
    bool more_places_to_split = true;
    while (more_places_to_split) {
      LOG(WARNING) << "[Places 2d Segmenter] starting split loop ";

      more_places_to_split = false;
      // for each semantic class, check "incompatible" semantic classes. for each impure
      // place, split it. here we repeatedly check all places, but it's possible to make
      // this faster by only checking "neighbor" places and being smarter about tracking
      // which splits can cause other places to need re-checking
      for (const auto place_label : config_.labels) {
        LOG(WARNING) << "[Places 2d Segmenter] analyzing label:  " << place_label;
        if (label_places.count(place_label) == 0) continue;
        std::vector<Place2d> updated_places;
        updated_places.clear();
        for (Place2d p : label_places.at(place_label)) {
          // !!! Need to push p to updated places????
          if (!p.can_split) {
            updated_places.push_back(p);
            continue;
          }
          if (!canSplit(p, config_)) {
            p.can_split = false;
            updated_places.push_back(p);
            continue;
          }

          LOG(WARNING) << "[Places 2d Segmenter] place with size: "
                       << p.cut_plane.norm();
          bool already_split = false;
          for (const auto impurity_label : config_.impurity_labels) {
            if (label_places.count(impurity_label) == 0) continue;
            if (place_label == impurity_label) {
              // reasonable to assume that there's no reason to split a place that only
              // intersects with its own type, even if it's an impurity
              continue;
            }
            for (Place2d ip : label_places.at(impurity_label)) {
              if (shouldImpurityCauseSplit(p, ip)) {
                std::pair<Place2d, Place2d> split_places =
                    splitPlace(full_mesh_vertices_->points, p);
                LOG(WARNING) << "[Places 2d Segmenter] split place!";
                updated_places.push_back(split_places.first);
                updated_places.push_back(split_places.second);
                already_split = true;
                more_places_to_split = true;
                break;
              }
            }
            if (already_split) break;
          }
          if (!already_split) {
            updated_places.push_back(p);
          }
        }
        label_places[place_label] = updated_places;
      }
    }
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

bool Place2dSegmenter::shouldImpurityCauseSplit(const Place2d& place,
                                                const Place2d& impurity) {
  // Currently we check for impurities with the same transverse logic as for adding
  // edges between places, but it would probably be better to do something volume-based
  Eigen::Vector2f ec1 = place.ellipse_centroid.head(2).cast<float>();
  Eigen::Vector2f ec2 = impurity.ellipse_centroid.head(2).cast<float>();
  double overlap_distance = get_ellipsoid_transverse_overlap_distance(
      place.ellipse_matrix_compress, ec1, impurity.ellipse_matrix_compress, ec2);
  double centroid_height_offset =
      std::abs(place.ellipse_centroid(2) - impurity.ellipse_centroid(2));
  // if (overlap_distance > config_.place_overlap_threshold && centroid_height_offset <
  // config_.place_max_neighbor_z_diff) {
  if (overlap_distance > 0 &&
      centroid_height_offset < config_.place_max_neighbor_z_diff) {
    return true;
  } else {
    return false;
  }
}

bool Place2dSegmenter::shouldAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                                                const Place2dNodeAttributes& attrs2,
                                                EdgeAttributes& edge_attrs) {
  Eigen::Vector3f ec1 = attrs1.ellipse_centroid;
  Eigen::Matrix<float, 2, 2> em1 = attrs1.ellipse_matrix_compress;

  Eigen::Vector3f ec2 = attrs2.ellipse_centroid;
  Eigen::Matrix<float, 2, 2> em2 = attrs2.ellipse_matrix_compress;

  double overlap_distance =
      get_ellipsoid_transverse_overlap_distance(em1, ec1.head(2), em2, ec2.head(2));
  double centroid_height_offset = std::abs(ec1(2) - ec2(2));
  edge_attrs.weight = overlap_distance;
  edge_attrs.weighted = true;
  if (overlap_distance > config_.place_overlap_threshold &&
      centroid_height_offset < config_.place_max_neighbor_z_diff) {
    return true;
  } else {
    return false;
  }
}

void Place2dSegmenter::updateGraph(uint64_t timestamp_ns,
                                   const ReconstructionOutput& msg,
                                   DynamicSceneGraph& graph) {
  // Remove old empty nodes
  for (const auto& nid : nodes_to_remove_) {
    graph.removeNode(nid);
  }
  nodes_to_remove_.clear();

  std::optional<Eigen::Vector3d> pos = msg.current_position;
  VLOG(1) << "[Places 2d Segmenter] updateGraph";
  std::map<uint32_t, std::set<NodeId>> new_active_places;
  for (const auto& label : config_.labels) {
    new_active_places[label] = std::set<NodeId>();
  }

  if (!pos) {
    new_active_places = active_places_;
  } else {
    for (auto kv : active_places_) {
      std::set<NodeId> nodes = kv.second;
      for (NodeId nid : nodes) {
        Place2dNodeAttributes& attrs =
            graph.getNode(nid)->get().attributes<Place2dNodeAttributes>();
        if (attrs.pcl_mesh_connections.size() == 0 || attrs.boundary.size() < 3) {
          // Remove dangling places
          graph.removeNode(nid);
          continue;
        }
        if (attrs.pcl_max_index < num_archived_vertices_) {
          // all mesh vertices in this place have been archived, so we can forget about
          // it.
          attrs.is_active = false;
        } else if (attrs.pcl_min_index < num_archived_vertices_) {
          // keep track of places that are close enough to affect future iterations
          new_active_places.at(kv.first).insert(nid);
        } else {
          // no archived vertices in place, so we can get rid of it
          graph.removeNode(nid);
        }
      }
    }
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
    Place2dNodeAttributes attrs1 =
        graph.getNode(ns1)->get().attributes<Place2dNodeAttributes>();

    for (NodeSymbol ns2 : full_nodes) {
      if (ns1 == ns2) {
        continue;
      }
      Place2dNodeAttributes attrs2 =
          graph.getNode(ns2)->get().attributes<Place2dNodeAttributes>();
      EdgeAttributes ea;
      if (shouldAddPlaceConnection(attrs1, attrs2, ea)) {
        graph.insertEdge(ns1, ns2, ea.clone());
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

  pcl::PointXYZ centroid;
  place.centroid.get(centroid);
  attrs->position << centroid.x, centroid.y, centroid.z;
  attrs->is_active = true;

  attrs->semantic_label = label;
  attrs->name = NodeSymbol(next_node_id_).getLabel();
  attrs->boundary = place.boundary;
  attrs->pcl_boundary_connections.insert(attrs->pcl_boundary_connections.begin(),
                                         place.boundary_indices.indices.begin(),
                                         place.boundary_indices.indices.end());
  attrs->ellipse_matrix_compress = place.ellipse_matrix_compress;
  attrs->ellipse_matrix_expand = place.ellipse_matrix_expand;
  attrs->ellipse_centroid(0) = place.ellipse_centroid(0);
  attrs->ellipse_centroid(1) = place.ellipse_centroid(1);
  attrs->ellipse_centroid(2) = centroid.z;
  attrs->pcl_min_index = place.min_mesh_index;
  attrs->pcl_max_index = place.max_mesh_index;

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

  graph.emplaceNode(DsgLayers::MESH_PLACES, next_node_id_, std::move(attrs));

  active_places_.at(label).insert(next_node_id_);
  active_place_timestamps_[next_node_id_] = timestamp;

  return next_node_id_++;
}

}  // namespace hydra
