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
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <kimera_pgmo/compression/delta_compression.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <filesystem>
#include <nanoflann.hpp>

#include "hydra/eval/progress.h"
#include "hydra/reconstruction/voxel_types.h"
#include "hydra/utils/pgmo_mesh_interface.h"
#include "hydra/utils/pgmo_mesh_traits.h"

DEFINE_double(mesh_resolution, 0.4, "mesh resolution in meters");
DEFINE_double(place_resolution, 1.5, "place resolution in meters");

namespace hydra::eval {

using namespace spark_dsg;

using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::L2_Simple_Adaptor;

struct GraphAdaptor {
  GraphAdaptor(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes)
      : nodes(nodes) {
    positions = Eigen::MatrixXd::Zero(3, nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
      positions.block<3, 1>(0, i) = layer.getPosition(nodes[i]);
    }
  }

  size_t kdtree_get_point_count() const { return nodes.size(); }

  double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return positions(dim, idx);
  }

  Eigen::Vector3d pos(const size_t idx) const { return positions.block<3, 1>(0, idx); }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  Eigen::MatrixXd positions;
  std::vector<NodeId> nodes;
};

struct CloudDistanceAdaptor {
  using ElementType = double;
  using IndexType = size_t;
  using DistanceType = double;

  CloudDistanceAdaptor(const GraphAdaptor& cloud) : cloud_(cloud) {}

  double evalMetric(const double* a, const size_t b_idx, size_t) const {
    const auto b_pos = cloud_.pos(b_idx);
    const auto dist = (Eigen::Map<const Eigen::Vector3d>(a) - b_pos).squaredNorm();
    return dist;
  }

  template <typename U, typename V>
  double accum_dist(const U a, const V b, const size_t) const {
    return (a - b) * (a - b);
  }

  const GraphAdaptor& cloud_;
};

struct RadiusSearcher {
 public:
  using Dist = CloudDistanceAdaptor;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, GraphAdaptor, 3, size_t>;

  explicit RadiusSearcher(const SceneGraphLayer& graph,
                          const std::vector<NodeId>& nodes)
      : adaptor_(graph, nodes) {
    kdtree_.reset(new KDTree(3, adaptor_));
    kdtree_->buildIndex();
    params_.sorted = true;
  }

  void find(const Eigen::Vector3d& pos,
            double radius_squared,
            std::vector<NodeId>& neighbors) const {
    std::vector<nanoflann::ResultItem<size_t, double>> results;
    const auto num_found =
        kdtree_->radiusSearch(pos.data(), radius_squared, results, params_);
    for (size_t i = 1; i < num_found; ++i) {
      neighbors.push_back(results[i].first);
    }
  }

 private:
  GraphAdaptor adaptor_;
  nanoflann::SearchParameters params_;
  std::unique_ptr<KDTree> kdtree_;
};

size_t getVertexFromFaces(const spark_dsg::Mesh& mesh, size_t i) {
  size_t face = i / 3;
  size_t face_idx = i % 3;
  return mesh.face(face)[face_idx];
}

void remapPlaces(DynamicSceneGraph& graph,
                 const std::unordered_map<size_t, size_t>& remapping) {
  const auto& places = graph.getLayer(DsgLayers::PLACES);
  for (auto&& [id, node] : places.nodes()) {
    VLOG(10) << "remapping place node: " << NodeSymbol(id).getLabel();

    auto& attrs = node->attributes<PlaceNodeAttributes>();
    for (auto& connection : attrs.pcl_mesh_connections) {
      const auto iter = remapping.find(connection);
      if (iter == remapping.end()) {
        continue;
      }

      connection = iter->second;
    }
  }
}

void compressMesh(DynamicSceneGraph& graph, double resolution) {
  kimera_pgmo::DeltaCompression compression(resolution);
  kimera_pgmo::HashedIndexMapping remapping;

  if (!graph.mesh()) {
    return;
  }
  PgmoMeshInterface interface(*graph.mesh());
  auto delta = compression.update(interface, 0, &remapping);

  auto& face_remapping = remapping.at(BlockIndex(0, 0, 0));
  std::unordered_map<size_t, size_t> vertex_remapping;
  for (auto&& [face_idx, new_idx] : face_remapping) {
    size_t vertex_idx = getVertexFromFaces(*graph.mesh(), face_idx);
    vertex_remapping[vertex_idx] = new_idx;
  }

  graph.setMesh(std::make_shared<spark_dsg::Mesh>());
  delta->updateMesh(*graph.mesh());
  remapPlaces(graph, vertex_remapping);
}

void compressPlaces(DynamicSceneGraph& graph, double resolution) {
  std::vector<NodeId> nodes;
  const auto& places = graph.getLayer(DsgLayers::PLACES);
  for (const auto& id_node_pair : places.nodes()) {
    nodes.push_back(id_node_pair.first);
  }

  const double threshold = resolution * resolution;
  RadiusSearcher search(places, nodes);

  std::vector<bool> merged(nodes.size(), false);
  std::list<std::vector<NodeId>> to_merge;

  std::vector<size_t> to_iter(nodes.size());
  std::iota(to_iter.begin(), to_iter.end(), 0);
  for (const auto i : progress::wrap(to_iter)) {
    if (merged.at(i)) {
      continue;
    }

    const auto pos_i = graph.getPosition(nodes[i]);
    std::vector<size_t> neighbors;
    search.find(pos_i, threshold, neighbors);

    std::vector<size_t> curr_merges;
    for (const auto j : neighbors) {
      if (merged.at(j)) {
        continue;
      }

      const auto pos_j = graph.getPosition(nodes[j]);
      if ((pos_i - pos_j).norm() >= resolution) {
        continue;
      }

      curr_merges.push_back(j);
    }

    if (curr_merges.empty()) {
      continue;
    }

    curr_merges.push_back(i);
    std::vector<NodeId> merge_ids;
    for (const auto idx : curr_merges) {
      merged[idx] = true;
      merge_ids.push_back(nodes[idx]);
    }

    to_merge.push_back(merge_ids);
  }

  for (const auto& merge : to_merge) {
    std::vector<PlaceNodeAttributes> prev_attrs;
    for (const auto& node_id : merge) {
      CHECK(graph.hasNode(node_id));
      prev_attrs.push_back(graph.getNode(node_id).attributes<PlaceNodeAttributes>());
    }

    for (size_t i = 1; i < merge.size(); ++i) {
      graph.mergeNodes(merge[i], merge[0]);
    }

    const auto& node = graph.getNode(merge[0]);
    auto& attrs = node.attributes<PlaceNodeAttributes>();
    attrs.distance = 0;
    attrs.position = Eigen::Vector3d::Zero();
    attrs.num_basis_points = 0;
    attrs.voxblox_mesh_connections.clear();
    attrs.pcl_mesh_connections.clear();
    std::set<size_t> seen_pcl;
    for (const auto& other : prev_attrs) {
      attrs.distance += other.distance;
      attrs.position += other.position;
      attrs.num_basis_points += other.num_basis_points;
      for (const auto idx : other.pcl_mesh_connections) {
        if (seen_pcl.count(idx)) {
          continue;
        }

        seen_pcl.insert(idx);
        attrs.pcl_mesh_connections.push_back(idx);
      }

      // TODO(nathan) filter by seen
      attrs.voxblox_mesh_connections.insert(attrs.voxblox_mesh_connections.end(),
                                            other.voxblox_mesh_connections.begin(),
                                            other.voxblox_mesh_connections.end());
    }

    attrs.distance /= prev_attrs.size();
    attrs.position /= prev_attrs.size();
    attrs.num_basis_points =
        static_cast<int>(std::round(attrs.num_basis_points / prev_attrs.size()));
  }
}

}  // namespace hydra::eval

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::SetUsageMessage("utility for comparing visualizing room bounding boxes");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  for (int i = 1; i < argc; ++i) {
    std::filesystem::path dsg_path =
        std::filesystem::canonical(std::filesystem::path(argv[i]));
    if (!std::filesystem::exists(dsg_path)) {
      LOG(WARNING) << "Skipping invalid path: '" << dsg_path.string() << "'";
    }

    LOG(INFO) << "Loading graph from '" << dsg_path.string() << "'";
    const auto graph = spark_dsg::DynamicSceneGraph::load(dsg_path.string());
    size_t num_places = graph->getLayer(spark_dsg::DsgLayers::PLACES).numNodes();
    size_t num_vertices = !graph->hasMesh() ? 0 : graph->mesh()->numVertices();
    LOG(INFO) << "Loaded graph with " << num_places << " places and " << num_vertices
              << " vertices";

    LOG(INFO) << "Compressing mesh...";
    hydra::eval::compressMesh(*graph, FLAGS_mesh_resolution);
    size_t new_vertices = !graph->hasMesh() ? 0 : graph->mesh()->numVertices();
    LOG(INFO) << "Compressed mesh to " << new_vertices << " vertices (from "
              << num_vertices << ")";

    LOG(INFO) << "Compressing places...";
    hydra::eval::compressPlaces(*graph, FLAGS_place_resolution);
    size_t new_places = graph->getLayer(spark_dsg::DsgLayers::PLACES).numNodes();
    LOG(INFO) << "Compressed places to " << new_places << " places (from " << num_places
              << ")";

    const auto out_path =
        dsg_path.replace_filename(dsg_path.stem().string() + "_compressed.json");
    LOG(INFO) << "Saving graph to '" << out_path.string() << "'";
    graph->save(out_path, true);
  }
  return 0;
}
