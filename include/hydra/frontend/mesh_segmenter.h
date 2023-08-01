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
#include "hydra/frontend/mesh_segmenter_config.h"

namespace hydra {

struct Cluster {
  using PointT = pcl::PointXYZRGBA;
  using CloudT = pcl::PointCloud<PointT>;
  using CentroidT = pcl::CentroidPoint<pcl::PointXYZ>;
  CentroidT centroid;
  CloudT::Ptr cloud;
  pcl::PointIndices indices;
};

class MeshSegmenter {
 public:
  using IndicesVector = pcl::IndicesPtr::element_type;
  using LabelIndices = std::map<uint32_t, pcl::IndicesPtr>;
  using MeshVertexCloud = Cluster::CloudT;
  using Clusters = std::vector<Cluster>;
  using LabelClusters = std::map<uint32_t, Clusters>;
  using CallbackFunc = std::function<void(const MeshVertexCloud& cloud,
                                          const IndicesVector& indices,
                                          const LabelIndices& label_indices)>;

  MeshSegmenter(const MeshSegmenterConfig& config,
                const MeshVertexCloud::Ptr& active_vertices,
                const std::shared_ptr<std::vector<uint32_t>>& labels);

  LabelClusters detect(const pcl::IndicesPtr& frontend_indices,
                       const std::optional<Eigen::Vector3d>& pos);

  inline std::unordered_set<NodeId> getObjectsToCheckForPlaces() const {
    return objects_to_check_for_places_;
  }

  void pruneObjectsToCheckForPlaces(const DynamicSceneGraph& graph);


  std::set<NodeId> updateGraph(DynamicSceneGraph& graph,
                               const LabelClusters& clusters,
                               uint64_t timestamp);

  inline void addVisualizationCallback(const CallbackFunc& func) {
    callback_funcs_.push_back(func);
  }

 private:
  Clusters findClusters(const MeshVertexCloud::Ptr& cloud,
                        const pcl::IndicesPtr& indices) const;

  std::set<NodeId> archiveOldObjects(const DynamicSceneGraph& graph,
                                     uint64_t latest_timestamp);

  LabelIndices getLabelIndices(const IndicesVector& indices) const;

  void addObjectToGraph(DynamicSceneGraph& graph,
                        const Cluster& cluster,
                        uint32_t label,
                        uint64_t timestamp);

  void updateObjectInGraph(const Cluster& cluster,
                           const SceneGraphNode& node,
                           uint64_t timestamp);

 private:
  MeshVertexCloud::Ptr full_mesh_vertices_;
  std::shared_ptr<std::vector<uint32_t>> full_mesh_labels_;

  MeshSegmenterConfig config_;
  NodeSymbol next_node_id_;

  std::map<uint32_t, std::set<NodeId>> active_objects_;
  std::map<NodeId, uint64_t> active_object_timestamps_;
  std::unordered_set<NodeId> objects_to_check_for_places_;
  std::vector<CallbackFunc> callback_funcs_;
};

}  // namespace hydra
