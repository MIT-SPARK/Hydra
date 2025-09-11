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
#include "hydra/places/traversability_clustering.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <iostream>
#include <queue>

#include "hydra/utils/timing_utilities.h"

namespace hydra::places {

using Timer = hydra::timing::ScopedTimer;
using spark_dsg::Boundary;
using spark_dsg::Side;
using spark_dsg::TraversabilityNodeAttributes;

void declare_config(BlockTraversabilityClustering::Config& config) {
  using namespace config;
  name("BlockTraversabilityClustering::Config");
  field(config.min_place_width, "min_place_width");
  field(config.max_place_width, "max_place_width");
  field(config.recursive, "recursive");
  field(config.simplify_boundary_traversability, "simplify_boundary_traversability");
  // Cognition_verifier parameters.
  field(config.project_labels_to_ground, "project_labels_to_ground");
  field(config.label_depth_tolerance, "label_depth_tolerance");
  field(config.label_use_const_weight, "label_use_const_weight");

  check(config.min_place_width, GT, 0, "min_place_width");
  check(config.max_place_width, GT, 0, "max_place_width");
  checkCondition(config.max_place_width > config.min_place_width,
                 "'max_place_width' must be greater than 'min_place_width'");
}

BlockTraversabilityClustering::BlockTraversabilityClustering(const Config& config)
    : config(config::checkValid(config)), infos_(config.max_place_width) {}

void BlockTraversabilityClustering::updateGraph(const TraversabilityLayer& layer,
                                                const ActiveWindowOutput& msg,
                                                spark_dsg::DynamicSceneGraph& graph) {
  // Compute all updated places and store them in the place_infos.
  current_time_ns_ = msg.timestamp_ns;
  updateInfoLayer(layer);
  computePlaces();
  classifyPlaceBoundaries();
  updatePlaceNodesInDsg(graph);
  updatePlaceEdgesInDsg(graph);
  archivePlaceInfos(graph);
  extractSemanticLabels(msg, graph);
}

void BlockTraversabilityClustering::updateInfoLayer(const TraversabilityLayer& layer) {
  Timer timer("traversability/clustering/update_layer", current_time_ns_);

  // Reset the updated and active flag for all info blocks.
  BlockIndexSet touched_blocks;
  for (const auto& block : layer) {
    const BlockIndices touched = touchedInfoBlocks(block);
    touched_blocks.insert(touched.begin(), touched.end());
  }
  for (auto& info : infos_) {
    info.updated = false;         // Traversability update.
    info.places_updated = false;  // Place update.
    info.active = touched_blocks.count(info.index);
  }

  // Compute and cache voxel resolutions.
  const size_t vps = std::ceil(config.max_place_width / layer.voxel_size);
  min_place_width_ = std::ceil(config.min_place_width / layer.voxel_size);
  min_component_size_ = min_place_width_ * min_place_width_;

  // Update and interpolate the traversability information in all info blocks.
  for (const auto block : layer.updatedBlocks()) {
    for (const auto& index : touchedInfoBlocks(*block)) {
      auto& info = infos_.allocateBlock(index, vps);
      // Find the offset from traversability block to info block coordinates.
      const Index2D offset =
          (info.index * vps - block->index * block->voxels_per_side).head<2>();
      const Index2D start = offset.cwiseMax(0);
      const Index2D end =
          (offset + Index2D::Ones() * vps).cwiseMin(block->voxels_per_side);
      for (int x = start.x(); x < end.x(); ++x) {
        for (int y = start.y(); y < end.y(); ++y) {
          info.voxel(x - offset.x(), y - offset.y()) = block->voxel(x, y);
        }
      }
      info.updated = true;
    }
  }
}

void BlockTraversabilityClustering::computePlaces() {
  Timer timer("traversability/clustering/compute_places", current_time_ns_);
  for (auto& info : infos_) {
    if (!info.updated) {
      continue;
    }

    // Find inscribing rectangle(s) for traversable voxels in the info block.
    std::vector<Range> rectangles = findRectangles(info);

    // Case all places were removed.
    if (rectangles.empty()) {
      if (!info.places.empty()) {
        for (auto& place : info.places) {
          place.deleted = true;
        }
        info.places_updated = true;
      }
      continue;
    }

    // Add or remove places in the info layer.
    if (info.places.size() > rectangles.size()) {
      for (size_t i = rectangles.size(); i < info.places.size(); ++i) {
        info.places[i].deleted = true;
      }
      info.places_updated = true;
    } else if (info.places.size() < rectangles.size()) {
      info.places.resize(rectangles.size());
      info.places_updated = true;
    }

    // Update the place info with the new rectangles.
    const double voxel_size = info.block_size / info.voxels_per_side;
    for (size_t i = 0; i < rectangles.size(); ++i) {
      auto& place = info.places[i];
      if (place.range == rectangles[i]) {
        // No change, keep the existing place info.
        continue;
      }
      info.places_updated = true;
      place.range = rectangles[i];
      place.boundary_info.min =
          Eigen::Vector2d(place.range.x_start, place.range.y_start) * voxel_size +
          info.origin().head<2>().cast<double>();
      place.boundary_info.max =
          Eigen::Vector2d(place.range.x_end + 1, place.range.y_end + 1) * voxel_size +
          info.origin().head<2>().cast<double>();
      info.free = place.range.area() == config.max_place_width * config.max_place_width;
      if (place.first_seen_time_ns == 0) {
        place.first_seen_time_ns = current_time_ns_;
      }
    }
  }
}

BlockTraversabilityClustering::Component
BlockTraversabilityClustering::traversableVoxels(
    const TraversabilityBlock& block) const {
  Index2DSet candidates;
  for (size_t x = 0; x < block.voxels_per_side; ++x) {
    for (size_t y = 0; y < block.voxels_per_side; ++y) {
      if (block.voxel(x, y).state == State::TRAVERSABLE) {
        candidates.emplace(x, y);
      }
    }
  }
  return candidates;
}

BlockTraversabilityClustering::Components
BlockTraversabilityClustering::connectedComponents(Component& candidates) const {
  // Simple BFS to find connected components in the candidates.
  static const std::vector<Index2D> offsets = {
      Index2D(1, 0), Index2D(-1, 0), Index2D(0, 1), Index2D(0, -1)};
  Components components;

  while (candidates.size() >= min_component_size_) {
    // Start BFS for a new component
    Component component;
    std::queue<Index2D> q;
    const Index2D start = *candidates.begin();
    q.emplace(start);
    candidates.erase(start);
    while (!q.empty()) {
      const Index2D current_index = q.front();
      q.pop();
      component.insert(current_index);

      // Check 4-connected neighbors
      for (const auto& offset : offsets) {
        const Index2D neighbor = current_index + offset;
        if (candidates.count(neighbor)) {
          candidates.erase(neighbor);
          q.emplace(neighbor);
        }
      }
    }

    if (component.size() >= min_component_size_) {
      components.emplace_back(std::move(component));
    }
  }
  return components;
}

std::optional<BlockTraversabilityClustering::Range>
BlockTraversabilityClustering::inscribeRectangle(const Component& component) const {
  // Find the minimum enclosing rectangle as bounds for where to search.
  Range rectangle{
      std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max(), 0, 0};
  for (const auto& index : component) {
    rectangle.x_start = std::min(rectangle.x_start, static_cast<size_t>(index.x()));
    rectangle.y_start = std::min(rectangle.y_start, static_cast<size_t>(index.y()));
    rectangle.x_end = std::max(rectangle.x_end, static_cast<size_t>(index.x()));
    rectangle.y_end = std::max(rectangle.y_end, static_cast<size_t>(index.y()));
  }

  // Check the max rectangle size and connectivity to the sides of the subblock.
  if (rectangle.width() < min_place_width_ || rectangle.height() < min_place_width_) {
    return std::nullopt;
  }

  // Problem is feasible. Check for the simplest case where the entire rectangle is
  // traversable.
  if (rectangle.area() == component.size()) {
    return rectangle;
  }

  // Find the largest rectangle that is completely traversable by computing the
  // largest admissible rectangle for each side.
  auto traversable = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(
                         rectangle.height(), rectangle.width())
                         .eval();
  for (const auto& index : component) {
    traversable(index.y() - rectangle.y_start, index.x() - rectangle.x_start) = true;
  }

  Range best_rectangle;
  size_t best_area = 0;
  size_t best_rotation = 0;
  for (const Side side : Side::ALL) {
    // Build the height histogram for the current side.
    const size_t width = traversable.cols();
    const size_t height = traversable.rows();
    Eigen::VectorXi heights = Eigen::VectorXi::Ones(width) * (height - 1);
    for (size_t col = 0; col < width; ++col) {
      for (size_t row = 0; row < height; ++row) {
        if (!traversable(row, col)) {
          heights(col) = row >= min_place_width_ ? row - 1 : 0;
          break;
        }
      }
    }

    // Find the largest rectangle in the local histogram.
    for (size_t start = 0; start <= width - min_place_width_; ++start) {
      // Check all rectangles that can be formed starting from the current column.
      if (!heights(start)) {
        continue;
      }
      const size_t remaining_width = width - start;
      if ((heights(start) + 1) * remaining_width <
          std::max(best_area + 1, min_component_size_)) {
        // If the maximal area cannot be the best, skip.
        continue;
      }
      Range candidate{start, 0, 0, std::numeric_limits<size_t>::max()};
      for (size_t c_width = 0; c_width < remaining_width; ++c_width) {
        candidate.y_end =
            std::min(candidate.y_end, static_cast<size_t>(heights(start + c_width)));
        if (!candidate.y_end) {
          break;
        }
        if (c_width < min_place_width_ - 1) {
          continue;
        }
        candidate.x_end = start + c_width;
        const size_t area = candidate.area();
        if (area > best_area) {
          best_area = area;
          best_rectangle = candidate;
          best_rotation = side;
        }
      }
    }
    // Rotate the traversable matrix counterclockwise to check the next side.
    traversable = traversable.transpose().colwise().reverse().eval();
  }
  if (best_area == 0 || best_rectangle.width() < min_place_width_ ||
      best_rectangle.height() < min_place_width_) {
    return std::nullopt;  // No traversable rectangle found.
  }

  for (size_t i = best_rotation; i > 0; --i) {
    // Rotate the rectangle clockwise back into the original frame.
    const size_t width = (i % 2 == 0 ? rectangle.height() : rectangle.width()) - 1;
    best_rectangle = Range(width - best_rectangle.y_end,
                           best_rectangle.x_start,
                           width - best_rectangle.y_start,
                           best_rectangle.x_end);
  }

  // Translate the rectangle back to the original block coordinates.
  best_rectangle.x_start += rectangle.x_start;
  best_rectangle.x_end += rectangle.x_start;
  best_rectangle.y_start += rectangle.y_start;
  best_rectangle.y_end += rectangle.y_start;

  return best_rectangle;
}

void BlockTraversabilityClustering::classifyPlaceBoundaries() {
  Timer timer("traversability/clustering/classify_boundaries", current_time_ns_);
  for (auto& info : infos_) {
    for (auto& place : info.places) {
      if (place.deleted) {
        continue;
      }
      // TODO(lschmid): Consider checking whether this and neighbor blocks have been
      // updated to shorten computation.
      const auto before = place.boundary_info;
      for (const Side side : Side::ALL) {
        // Classify each side of the place boundary [bottom, left, top, right].
        std::vector<State> states;

        // 1. If the place is not connected to the end of the block, check the
        // remainder.
        const size_t d_within = place.range.distanceToSide(side, info.voxels_per_side);
        if (d_within > 0) {
          const auto range = place.range.invertToSide(
              side, info.voxels_per_side, std::min(d_within, min_place_width_));
          states = computeBoundaryTraversability(info, range, side.vertical());
        }

        // 2. If needed, check connectivity to neighbor block.
        if (d_within < min_place_width_) {
          const size_t width =
              (side.horizontal() ? place.range.width() : place.range.height());
          const BlockIndex n_index = info.index + neighbors_[side];
          const auto n_block = infos_.getBlockPtr(n_index);
          if (!n_block) {
            // Completely unobserved neighbor.
            if (states.empty()) {
              states.resize(width, State::UNKNOWN);
            } else {
              fuseStates(State::UNKNOWN, states, false);
            }
          } else {
            // Neighbor block exists.
            if (n_block->free) {
              if (states.empty()) {
                states.resize(width, State::TRAVERSABLE);
              } else {
                fuseStates(State::TRAVERSABLE, states, false);
              }
            } else {
              const auto range = place.range.projectToNextBlock(
                  side, info.voxels_per_side, min_place_width_ - d_within);
              const auto next_states =
                  computeBoundaryTraversability(*n_block, range, side.vertical());
              if (states.empty()) {
                states = std::move(next_states);
              } else {
                fuseStates(next_states, states, false);
              }
            }
          }
        }

        // Write the results into the info.
        if (!config.simplify_boundary_traversability) {
          place.boundary_info.states[side] = std::move(states);
          continue;
        }

        // Summarize the traversability states into a single value.
        const auto min_max_traversability =
            spark_dsg::computeMinMaxTraversability(states);
        if (min_max_traversability.first >= min_place_width_) {
          place.boundary_info.states[side] = {State::TRAVERSABLE};
        } else if (min_max_traversability.second < min_place_width_) {
          place.boundary_info.states[side] = {State::INTRAVERSABLE};
        } else {
          place.boundary_info.states[side] = {State::UNKNOWN};
        }
      }

      if (place.boundary_info != before) {
        info.places_updated = true;
      }
    }
  }
}

std::vector<BlockTraversabilityClustering::State>
BlockTraversabilityClustering::computeBoundaryTraversability(
    const TraversabilityBlock& block, const Range& range, bool vertical) const {
  std::vector<State> states;
  if (vertical) {
    // Check the vertical boundary.
    states.resize(range.height(), State::TRAVERSABLE);
    for (size_t y = range.y_start; y <= range.y_end; ++y) {
      for (size_t x = range.x_start; x <= range.x_end; ++x) {
        const auto& voxel = block.voxel(x, y);
        if (voxel.state == State::INTRAVERSABLE) {
          states[y - range.y_start] = State::INTRAVERSABLE;
          break;
        } else if (voxel.state == State::UNKNOWN) {
          states[y - range.y_start] = State::UNKNOWN;
        }
      }
    }
  } else {
    // Check the horizontal boundary.
    states.resize(range.width(), State::TRAVERSABLE);
    for (size_t x = range.x_start; x <= range.x_end; ++x) {
      for (size_t y = range.y_start; y <= range.y_end; ++y) {
        const auto& voxel = block.voxel(x, y);
        if (voxel.state == State::INTRAVERSABLE) {
          states[x - range.x_start] = State::INTRAVERSABLE;
          break;
        } else if (voxel.state == State::UNKNOWN) {
          states[x - range.x_start] = State::UNKNOWN;
        }
      }
    }
  }
  return states;
}

bool BlockTraversabilityClustering::shouldConnect(const Boundary& from,
                                                  const Boundary& to,
                                                  Side side) const {
  // Check perpendicular distance.
  const double distance =
      std::abs(from.getCoordinate(side) - to.getCoordinate(side.opposite()));
  if (distance > config.min_place_width) {
    return false;
  }
  if (config.recursive && distance > config.min_place_width - 0.05) {
    // TODO(lschmid): This is supposed to prevent connections across thin places, but
    // there should be a better solution.
    return false;
  }

  // Check traversable width.
  const double width = from.maxTraversableDistance(to, side);
  return width >= config.min_place_width;
}

void BlockTraversabilityClustering::updateDsgEdge(
    const PlaceInfo& from,
    const PlaceInfo& to,
    bool should_connect,
    spark_dsg::DynamicSceneGraph& graph) const {
  if (should_connect) {
    // TODO(lschmid): For now use the weights to indicate that the edge is an
    // original active window edge when w=0.5.
    graph.addOrUpdateEdge(
        from.node_id, to.node_id, std::make_unique<spark_dsg::EdgeAttributes>(0.5));
  } else {
    graph.removeEdge(from.node_id, to.node_id);
  }
}

void BlockTraversabilityClustering::updatePlaceNodesInDsg(
    spark_dsg::DynamicSceneGraph& graph) {
  for (auto& info : infos_) {
    if (!info.places_updated) {
      continue;
    }
    for (auto it = info.places.begin(); it != info.places.end();) {
      auto& place = *it;
      if (place.deleted) {
        // Remove the place from the DSG if it has been deleted.
        if (place.node_id != 0) {
          graph.removeNode(place.node_id);
        }
        it = info.places.erase(it);
        continue;
      }

      if (place.node_id == 0) {
        // Node does not exist yet, create a new place node.
        place.node_id = spark_dsg::NodeSymbol('t', current_id_++);
        auto attrs = std::make_unique<spark_dsg::TraversabilityNodeAttributes>();
        attrs->is_active = true;
        graph.emplaceNode(
            spark_dsg::DsgLayers::TRAVERSABILITY, place.node_id, std::move(attrs));
      }

      // Update the place attributes.
      auto& attrs = graph.getNode(place.node_id)
                        .attributes<spark_dsg::TraversabilityNodeAttributes>();
      updatePlaceNodeAttributes(attrs, place);
      ++it;
    }
  }
}

void BlockTraversabilityClustering::updatePlaceNodeAttributes(
    spark_dsg::TraversabilityNodeAttributes& attrs, const PlaceInfo& place) {
  // Boundary positions relative to the block origin.
  Boundary(place.boundary_info).toAttributes(attrs);

  // General attributes.
  attrs.last_update_time_ns = current_time_ns_;
  attrs.first_observed_ns = place.first_seen_time_ns;
  attrs.last_observed_ns = current_time_ns_;
  attrs.is_active = true;
}

void BlockTraversabilityClustering::updatePlaceEdgesInDsg(
    spark_dsg::DynamicSceneGraph& graph) {
  for (const auto& info : infos_) {
    if (!info.places_updated) {
      continue;
    }
    // Note that there are no deleted or unallocated places at this point.
    for (const auto& place : info.places) {
      const Boundary from(place.boundary_info);

      // Candidates in the same block.
      for (const auto& n_place : info.places) {
        if (n_place.node_id == place.node_id) {
          continue;
        }
        const Boundary to(n_place.boundary_info);
        const auto side = from.isOnSide(to);
        if (!side.valid()) {
          updateDsgEdge(place, n_place, false, graph);
          continue;
        }
        const bool should_connect = shouldConnect(from, to, side);
        updateDsgEdge(place, n_place, should_connect, graph);
      }

      // Candidates in neighboring blocks.
      for (const Side side : Side::ALL) {
        const BlockIndex n_index = info.index + neighbors_[side];
        const auto n_info = infos_.getBlockPtr(n_index);
        if (!n_info) {
          continue;
        }
        for (const auto& n_place : n_info->places) {
          const Boundary to(n_place.boundary_info);
          const bool should_connect = shouldConnect(from, to, side);
          updateDsgEdge(place, n_place, should_connect, graph);
        }
      }
    }
  }
}

void BlockTraversabilityClustering::archivePlaceInfos(
    spark_dsg::DynamicSceneGraph& graph) {
  // Remove all place info blocks that are not in the current active window.
  BlockIndices to_remove;
  for (auto& info : infos_) {
    if (info.active) {
      continue;
    }

    // Deactivate all places in deactivated blocks.
    for (const auto& place : info.places) {
      if (place.node_id != 0) {
        graph.getNode(place.node_id)
            .attributes<spark_dsg::TraversabilityNodeAttributes>()
            .is_active = false;
      }
    }
    info.places.clear();

    // Check if there are neighboring blocks that depend on this block for boundary
    // computation.
    bool has_neighbor = false;
    for (const auto& offset : neighbors_) {
      const BlockIndex n_index = info.index + offset;
      const auto n_block = infos_.getBlockPtr(n_index);
      if (n_block && n_block->active) {
        has_neighbor = true;
        break;
      }
    }
    if (!has_neighbor) {
      to_remove.emplace_back(info.index);
    }
  }
  infos_.removeBlocks(to_remove);
};

void BlockTraversabilityClustering::extractSemanticLabels(
    const ActiveWindowOutput& msg, spark_dsg::DynamicSceneGraph& graph) {
  const auto& tsdf = msg.map().getTsdfLayer();
  const auto& sensor = msg.sensor_data->getSensor();
  const Eigen::Isometry3f sensor_T_world =
      msg.sensor_data->getSensorPose().cast<float>().inverse();

  for (const auto& [id, node] :
       graph.getLayer(spark_dsg::DsgLayers::TRAVERSABILITY).nodes()) {
    auto& attrs = node->attributes<spark_dsg::TraversabilityNodeAttributes>();
    if (!attrs.is_active) {
      continue;
    }

    // Find the intersection with the floor of the 3D point.
    Eigen::Vector3f pos_W = attrs.position.cast<float>();
    while (config.project_labels_to_ground) {
      const auto voxel = tsdf.getVoxelPtr(pos_W);
      if (!voxel) {
        break;
      }
      if (voxel->distance <= 0.0) {
        break;
      }
      pos_W.z() -= tsdf.voxel_size;
    }

    // Project to semantic frame, check is visible, and get ID.
    const Eigen::Vector3f pos_C = sensor_T_world * pos_W;
    int u, v;
    if (!sensor.projectPointToImagePlane(pos_C, u, v)) {
      continue;
    }
    const float img_range = msg.sensor_data->depth_image.at<InputData::RangeType>(v, u);
    const float place_range = pos_C.norm();
    if (place_range > img_range + config.label_depth_tolerance) {
      continue;
    }
    const int label = msg.sensor_data->label_image.at<InputData::LabelType>(v, u);
    float weight = 1.0f;
    if (!config.label_use_const_weight) {
      weight /= (place_range * place_range);  // Inverse square distance weighting.
    }
    attrs.cognition_labels[label] += weight;
  }
}

BlockIndices BlockTraversabilityClustering::touchedInfoBlocks(
    const TraversabilityBlock& block) const {
  const BlockIndex min = infos_.getBlockIndex(block.origin());
  const BlockIndex max =
      infos_.getBlockIndex(block.origin() + Point::Ones() * block.block_size);
  BlockIndices touched_blocks;
  for (auto x = min.x(); x <= max.x(); ++x) {
    for (auto y = min.y(); y <= max.y(); ++y) {
      touched_blocks.emplace_back(x, y, 0);
    }
  }
  return touched_blocks;
}

std::vector<BlockTraversabilityClustering::Range>
BlockTraversabilityClustering::findRectangles(const TraversabilityBlock& block) const {
  std::vector<Range> rectangles;
  auto candidates = traversableVoxels(block);
  Components components = connectedComponents(candidates);
  while (!components.empty()) {
    Component component = std::move(components.front());
    components.erase(components.begin());
    const auto rectangle = inscribeRectangle(component);
    if (!rectangle) {
      continue;
    }
    rectangles.emplace_back(std::move(*rectangle));
    if (config.recursive) {
      // Optionally search for additional rectangles in the same component.
      for (auto it = component.begin(); it != component.end();) {
        if (rectangle->contains(*it)) {
          it = component.erase(it);
        } else {
          ++it;
        }
      }
      const auto subcomponents = connectedComponents(component);
      components.insert(components.end(), subcomponents.begin(), subcomponents.end());
    }
  }
  return rectangles;
}

std::string BlockTraversabilityClustering::Range::toString() const {
  return "<[" + std::to_string(x_start) + ", " + std::string(std::to_string(x_end)) +
         "]x[" + std::to_string(y_start) + ", " + std::to_string(y_end) + "]>";
}

bool BlockTraversabilityClustering::Range::operator==(const Range& other) const {
  return x_start == other.x_start && y_start == other.y_start && x_end == other.x_end &&
         y_end == other.y_end;
}

BlockTraversabilityClustering::Range BlockTraversabilityClustering::Range::invertToSide(
    Side side, size_t voxels_per_side, size_t width) const {
  Range result = *this;
  switch (side) {
    case Side::BOTTOM:
      result.y_end = y_start - 1;
      result.y_start = y_start > width ? y_start - width : 0;
      return result;
    case Side::LEFT:
      result.x_end = x_start - 1;
      result.x_start = x_start > width ? x_start - width : 0;
      return result;
    case Side::TOP:
      result.y_start = y_end + 1;
      result.y_end = std::min(y_end + width, voxels_per_side);
      return result;
    case Side::RIGHT:
      result.x_start = x_end + 1;
      result.x_end = std::min(x_end + width, voxels_per_side);
      return result;
    default:
      throw std::out_of_range("Invalid side index for inverting range.");
  }
}

BlockTraversabilityClustering::Range
BlockTraversabilityClustering::Range::projectToNextBlock(Side side,
                                                         size_t voxels_per_side,
                                                         size_t width) const {
  Range result = *this;
  switch (side) {
    case Side::BOTTOM:
      result.y_end = voxels_per_side - 1;
      result.y_start = voxels_per_side - width;
      return result;
    case Side::LEFT:
      result.x_end = voxels_per_side - 1;
      result.x_start = voxels_per_side - width;
      return result;
    case Side::TOP:
      result.y_start = 0;
      result.y_end = width - 1;
      return result;
    case Side::RIGHT:
      result.x_start = 0;
      result.x_end = width - 1;
      return result;
    default:
      throw std::out_of_range("Invalid side index for extending range.");
  }
}

bool BlockTraversabilityClustering::Range::contains(const Index2D& index) const {
  return index.x() >= static_cast<int>(x_start) &&
         index.x() <= static_cast<int>(x_end) &&
         index.y() >= static_cast<int>(y_start) && index.y() <= static_cast<int>(y_end);
}

size_t BlockTraversabilityClustering::Range::distanceToSide(
    Side side, size_t voxels_per_side) const {
  switch (side) {
    case Side::BOTTOM:
      return y_start;
    case Side::LEFT:
      return x_start;
    case Side::TOP:
      return voxels_per_side - 1 - y_end;
    case Side::RIGHT:
      return voxels_per_side - 1 - x_end;
    default:
      throw std::out_of_range("Invalid side index for distance calculation.");
  }
}

}  // namespace hydra::places
