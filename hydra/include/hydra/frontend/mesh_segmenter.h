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
#include <spark_dsg/node_symbol.h>

#include "hydra/active_window/active_window_output.h"
#include "hydra/common/output_sink.h"
#include "hydra/frontend/graph_builder_functor.h"
#include "hydra/frontend/mesh_clustering.h"
#include "hydra/frontend/mesh_connection_updater.h"

namespace hydra {

using clustering::LabelIndices;

// A block pointer is an opaque revision token, not a persistent mesh index.
// The active cache or input packet owns it until the post-callback connection pass.
struct ObjectMeshVertex {
  const MeshBlock* block = nullptr;
  size_t vertex = 0;
};

class MeshSegmenter : public GraphBuilderFunctor {
 public:
  struct Cluster {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    std::vector<size_t> indices;
  };
  using LabelClusters = std::map<uint32_t, std::vector<Cluster>>;
  using Sink = OutputSink<uint64_t,
                          const spark_dsg::Mesh&,
                          const LabelIndices&,
                          const LabelClusters&>;

  struct Config {
    std::string layer_id = spark_dsg::DsgLayers::OBJECTS;
    clustering::VoxelClusteringConfig clustering;
    //! Tolerance to merge same-label vertices across active blocks (0 uses exact
    //! equality)
    double vertex_merge_tolerance_m = 1.0e-5;
    spark_dsg::BoundingBox::Type bounding_box_type = spark_dsg::BoundingBox::Type::AABB;
    // Association uses same-label spatial support within this radius.
    double association_tolerance = 0.25;
    double min_overlap_ratio = 0.2;
    std::vector<Sink::Factory> sinks;
  } const config;

  struct Object {
    spark_dsg::NodeId id;
    uint32_t label;
    uint64_t timestamp_ns = 0;
    bool is_active = true;
    bool has_archived = false;
    std::vector<ObjectMeshVertex> vertices;
    // Only support newly archived during this call. Resolved by the caller.
    std::vector<ObjectMeshVertex> archived_vertices;
    std::vector<Eigen::Vector3f> points;
  };

  explicit MeshSegmenter(const Config& config);
  MeshSegmenter(const Config& config, const std::set<uint32_t>& labels);

  void call(const ActiveWindowOutput& msg,
            SharedDsgInfo& dsg,
            FrontendOutput& output,
            const VolumetricWindow* window) override;

  void callPostUpdate(SharedDsgInfo& dsg,
                      FrontendOutput& output,
                      const MeshUpdateInfo& info) override;

  // Input block ownership is retained through connection resolution.
  void update(const ActiveWindowOutput& input);
  const std::map<spark_dsg::NodeId, Object>& objects() const { return objects_; }
  const std::vector<std::pair<spark_dsg::NodeId, spark_dsg::NodeId>>& merges() const {
    return merges_;
  }
  std::unordered_set<spark_dsg::NodeId> getActiveNodes() const;

 private:
  struct Detection;
  Detection prepareSamples() const;
  void cluster(uint64_t timestamp_ns);
  void associate(uint64_t timestamp_ns, const Detection& detection);
  spark_dsg::NodeSymbol next_node_id_;
  std::set<uint32_t> labels_;
  spatial_hash::IndexHashMap<MeshBlock::ConstPtr> blocks_;
  std::map<spark_dsg::NodeId, Object> objects_;
  std::vector<std::pair<spark_dsg::NodeId, spark_dsg::NodeId>> merges_;
  std::vector<MeshBlock::ConstPtr> retired_blocks_;
  Sink::List sinks_;
  MeshConnectionUpdater connections_;
};

void declare_config(MeshSegmenter::Config& config);

}  // namespace hydra
