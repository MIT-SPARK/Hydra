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
#pragma once

#include <config_utilities/factory.h>
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/traversability_boundary.h>

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "hydra/active_window/active_window_output.h"
#include "hydra/places/traversability_layer.h"

namespace hydra::places {

class TraversabilityClustering {
 public:
  using Ptr = std::shared_ptr<TraversabilityClustering>;
  using ConstPtr = std::shared_ptr<const TraversabilityClustering>;

  TraversabilityClustering() = default;
  virtual ~TraversabilityClustering() = default;

  virtual void updateGraph(const TraversabilityLayer& layer,
                           const ActiveWindowOutput& msg,
                           spark_dsg::DynamicSceneGraph& graph) = 0;
};

/**
 * @brief Simple clustering that assigns places by block.
 */
class BlockTraversabilityClustering : public TraversabilityClustering {
 public:
  struct Config {
    //! Minimum width of a place in meters.
    float min_place_width = 0.5f;

    //! Maximum width of a place in meters. This defines the place block size.
    float max_place_width = 1.0f;

    //! If true, identify multiple connected places in the same block where
    //! possible.
    bool recursive = false;

    //! If true, simplify the traversability of each boundary to a single value.
    //! Otherwise classify each voxel.
    bool simplify_boundary_traversability = true;

    // TMP cognition_verifier parameters.
    // If true project the labels to the ground using the TSDF.
    bool project_labels_to_ground = true;
    // If >0, use a fixed height above the sensor pose for projecting labels [m].
    float robot_height = -1.0f;
    // Maximum depth difference to check for occlusions when projecting labels [m].
    float label_depth_tolerance = 1.0f;
    // True: use inverse depth squared. False: use counting.
    bool label_use_const_weight = false;
  };

  using State = spark_dsg::TraversabilityState;
  using Component = Index2DSet;
  using Components = std::vector<Component>;

  struct Range {
    explicit Range(size_t x_start = 0,
                   size_t y_start = 0,
                   size_t x_end = 0,
                   size_t y_end = 0)
        : x_start(x_start), y_start(y_start), x_end(x_end), y_end(y_end) {}
    // Voxels in traversability block coordinates. Corner points are inclusive.
    size_t x_start;  // Start is bottom left corner in pixels.
    size_t y_start;
    size_t x_end;  // End is top right corner in pixels.
    size_t y_end;

    bool operator==(const Range& other) const;
    size_t width() const { return x_end - x_start + 1; }
    size_t height() const { return y_end - y_start + 1; }
    size_t area() const { return width() * height(); }
    std::string toString() const;
    Range invertToSide(spark_dsg::Side side,
                       size_t voxels_per_side,
                       size_t width) const;
    Range projectToNextBlock(spark_dsg::Side side,
                             size_t voxels_per_side,
                             size_t width) const;
    bool contains(const Index2D& index) const;
    size_t distanceToSide(spark_dsg::Side side, size_t voxels_per_side) const;
  };

  /**
   * @brief Information about a place node that is going to be created in the DSG.
   */
  struct PlaceInfo {
    //! Node ID in the DSG. 0 means not added to the DSG yet.
    spark_dsg::NodeId node_id = 0;

    //! Range of the place in the traversability block.
    Range range;

    //! Boundary information for each side [bottom, left, top, right].
    spark_dsg::BoundaryInfo boundary_info;

    //! First time this place was observed in the active window.
    uint64_t first_seen_time_ns = 0;

    //! Whether the place has been deleted and should be removed from the DSG.
    bool deleted = false;
  };

  struct InfoBlock : public TraversabilityBlock {
    InfoBlock(float block_size, const BlockIndex& index, size_t voxels_per_side)
        : TraversabilityBlock(block_size, index, voxels_per_side) {}

    // Places in the subblock.
    std::vector<PlaceInfo> places;

    // Whether the info is still in the active window.
    bool active = true;

    // Whether the block has updated place info.
    bool places_updated = false;

    // Whether the block is completely covered in a single free place.
    bool free = false;
  };
  using InfoLayer = spatial_hash::BlockLayer<InfoBlock>;

  BlockTraversabilityClustering(const Config& config);
  ~BlockTraversabilityClustering() = default;

  void updateGraph(const TraversabilityLayer& layer,
                   const ActiveWindowOutput& msg,
                   spark_dsg::DynamicSceneGraph& graph) override;

  const Config config;

 protected:
  /* Processing steps */
  void updateInfoLayer(const TraversabilityLayer& layer);
  void computePlaces();
  void classifyPlaceBoundaries();

  // Extract places and add to DSG.
  void updatePlaceNodesInDsg(spark_dsg::DynamicSceneGraph& graph);

  void updatePlaceEdgesInDsg(spark_dsg::DynamicSceneGraph& graph);

  // Archive exiting places in the Active Window.
  void archivePlaceInfos(spark_dsg::DynamicSceneGraph& graph);

  /* Utility functions */
  // Find all info block indices that would overlap with the given traversability block.
  BlockIndices touchedInfoBlocks(const TraversabilityBlock& block) const;

  // Find all inscribing rectangles in the block.
  std::vector<Range> findRectangles(const TraversabilityBlock& block) const;

  // Get all traversable voxels in the block.
  Component traversableVoxels(const TraversabilityBlock& block) const;

  // Cluster all candidate voxels into connected components.
  Components connectedComponents(Component& candidates) const;

  // Find the largest completely traversable rectangle in a component. Returns min and
  // max corners.
  std::optional<Range> inscribeRectangle(const Component& components) const;

  // Compute the traversable rows/columns in the range, over the vertical or
  // horizontal axis.
  std::vector<State> computeBoundaryTraversability(const TraversabilityBlock& block,
                                                   const Range& range,
                                                   bool vertical) const;

  // Update the scene graph place node attributes.
  void updatePlaceNodeAttributes(spark_dsg::TraversabilityNodeAttributes& attrs,
                                 const PlaceInfo& place);

  bool shouldConnect(const spark_dsg::Boundary& from,
                     const spark_dsg::Boundary& to,
                     spark_dsg::Side side) const;

  void updateDsgEdge(const PlaceInfo& from,
                     const PlaceInfo& to,
                     bool should_connect,
                     spark_dsg::DynamicSceneGraph& graph) const;

  /** TMP(lschmid): Currently added for Cognition verifier */
  // Extract semantic place features for each node from the input image.
  void extractSemanticLabels(const ActiveWindowOutput& msg,
                             spark_dsg::DynamicSceneGraph& graph);

  // Data.
  InfoLayer infos_;
  size_t current_id_ = 0;
  uint64_t current_time_ns_ = 0;
  double current_robot_height_ = 0.0;

  // Cached constants.
  // Minimum sizes in voxels.
  size_t min_place_width_;
  size_t min_component_size_;

  inline static const std::array<BlockIndex, 4> neighbors_ = {
      BlockIndex(0, -1, 0),  // bottom
      BlockIndex(-1, 0, 0),  // left
      BlockIndex(0, 1, 0),   // top
      BlockIndex(1, 0, 0)    // right
  };

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<TraversabilityClustering,
                                     BlockTraversabilityClustering,
                                     Config>("BlockTraversabilityClustering");
};

void declare_config(BlockTraversabilityClustering::Config& config);

}  // namespace hydra::places
