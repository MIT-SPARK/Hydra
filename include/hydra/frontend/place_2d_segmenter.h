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
#include <pcl/common/centroid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <memory>

#include "hydra/common/dsg_types.h"
#include "hydra/frontend/place_2d_segmenter_config.h"
#include "hydra/frontend/place_extractor_interface.h"
#include "opencv2/imgproc.hpp"

namespace hydra {

struct Place2d {
  using PointT = pcl::PointXYZRGBA;
  using CloudT = pcl::PointCloud<PointT>;
  using CentroidT = pcl::CentroidPoint<pcl::PointXYZ>;
  CentroidT centroid;
  pcl::PointIndices indices;
  pcl::PointIndices boundary_indices;
  std::vector<Eigen::Vector3d> boundary;
  Eigen::Matrix<float, 2, 2> ellipse_matrix_compress;
  Eigen::Matrix<float, 2, 2> ellipse_matrix_expand;
  Eigen::Vector2d ellipse_centroid;
  Eigen::Vector2d cut_plane;
  bool can_split;
};

void addPlaceRectInfo(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        points,
    Place2d& place);
void addPlaceBoundaryInfo(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        points,
    Place2d& place);
std::pair<Place2d, Place2d> splitPlace(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        points,
    const Place2d& place);

std::vector<Place2d> decomposePlaces(const Place2d::CloudT::Ptr cloud,
                                     const std::vector<Place2d>& initial_places,
                                     double min_size,
                                     size_t min_points);

std::vector<Place2d> decomposePlace(
    const std::vector<Place2d::PointT, Eigen::aligned_allocator<pcl::PointXYZRGBA>>&
        cloud_pts,
    const Place2d& place,
    const double min_size,
    const size_t min_points);

class Place2dSegmenter : public PlaceExtractorInterface {
 public:
  using IndicesVector = pcl::IndicesPtr::element_type;
  using LabelIndices = std::map<uint32_t, pcl::IndicesPtr>;
  using MeshVertexCloud = Place2d::CloudT;
  using Places = std::vector<Place2d>;
  using LabelPlaces = std::map<uint32_t, Places>;
  using CallbackFunc = std::function<void(const MeshVertexCloud& cloud,
                                          const IndicesVector& indices,
                                          const LabelIndices& label_indices)>;

  Place2dSegmenter(const Place2dSegmenterConfig& config,
                   const MeshVertexCloud::Ptr& active_vertices,
                   const std::shared_ptr<std::vector<uint32_t>>& labels);

  void detect(const ReconstructionOutput& msg,
              const kimera_pgmo::MeshDelta::Ptr& mesh_delta,
              const DynamicSceneGraph& graph) override;

  NodeIdSet getActiveNodes() const override;

  void updateGraph(uint64_t timestamp_ns,
                   const ReconstructionOutput&,
                   DynamicSceneGraph& graph) override;

  inline void addVisualizationCallback(const CallbackFunc& func) {
    callback_funcs_.push_back(func);
  }

 private:
  bool shouldImpurityCauseSplit(const Place2d& place, const Place2d& impurity);
  bool shouldAddPlaceConnection(const Place2dNodeAttributes& attrs1,
                                const Place2dNodeAttributes& attrs2,
                                EdgeAttributes& edge_weight);
  Places findPlaces(const MeshVertexCloud::Ptr& cloud,
                    const pcl::IndicesPtr& indices) const;

  std::set<NodeId> archiveOldObjects(const DynamicSceneGraph& graph,
                                     uint64_t latest_timestamp);

  LabelIndices getLabelIndices(const IndicesVector& indices) const;

  NodeSymbol addPlaceToGraph(DynamicSceneGraph& graph,
                             const Place2d& place,
                             uint32_t label,
                             uint64_t timestamp);

  void updatePlaceInGraph(const Place2d& place,
                          const SceneGraphNode& node,
                          uint64_t timestamp);

 private:
  MeshVertexCloud::Ptr full_mesh_vertices_;
  std::shared_ptr<std::vector<uint32_t>> full_mesh_labels_;
  LabelPlaces detected_label_places_;

  Place2dSegmenterConfig config_;
  NodeSymbol next_node_id_;

  std::map<uint32_t, std::set<NodeId>> active_places_;
  std::map<NodeId, uint64_t> active_place_timestamps_;
  std::vector<CallbackFunc> callback_funcs_;
};

}  // namespace hydra
