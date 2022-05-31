#include "hydra_topology/nearest_neighbor_utilities.h"
#include "hydra_topology/graph_extraction_utilities.h"

#include <nanoflann.hpp>

namespace hydra {
namespace topology {

using nanoflann::KDTreeSingleIndexAdaptor;
using nanoflann::KDTreeSingleIndexDynamicAdaptor;
using nanoflann::L2_Simple_Adaptor;
using GlobalIndexVector = voxblox::AlignedVector<GlobalIndex>;

struct GraphKdTreeAdaptor {
  GraphKdTreeAdaptor(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes)
      : layer(layer), nodes(nodes) {}

  inline size_t kdtree_get_point_count() const { return nodes.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return layer.getPosition(nodes[idx])(dim);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  const SceneGraphLayer& layer;
  std::vector<NodeId> nodes;
};

struct NearestNodeFinder::Detail {
  using Dist = L2_Simple_Adaptor<double, GraphKdTreeAdaptor>;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, GraphKdTreeAdaptor, 3, size_t>;

  Detail(const SceneGraphLayer& layer, const std::vector<NodeId>& nodes)
      : adaptor(layer, nodes) {
    kdtree.reset(new KDTree(3, adaptor));
    kdtree->buildIndex();
  }

  ~Detail() = default;

  GraphKdTreeAdaptor adaptor;
  std::unique_ptr<KDTree> kdtree;
};

NearestNodeFinder::NearestNodeFinder(const SceneGraphLayer& layer,
                                     const std::vector<NodeId>& nodes)
    : internals_(new Detail(layer, nodes)) {}

NearestNodeFinder::NearestNodeFinder(const SceneGraphLayer& layer,
                                     const std::unordered_set<NodeId>& nodes) {
  std::vector<NodeId> node_vector(nodes.begin(), nodes.end());
  internals_.reset(new Detail(layer, node_vector));
}

NearestNodeFinder::~NearestNodeFinder() {}

void NearestNodeFinder::find(const Eigen::Vector3d& position,
                             size_t num_to_find,
                             bool skip_first,
                             const NearestNodeFinder::Callback& callback) {
  std::vector<size_t> nn_indices(num_to_find);
  std::vector<double> distances(num_to_find);

  size_t num_found = internals_->kdtree->knnSearch(
      position.data(), num_to_find, nn_indices.data(), distances.data());

  size_t i = skip_first ? 1 : 0;
  for (i = 0; i < num_found; ++i) {
    callback(internals_->adaptor.nodes[nn_indices[i]], nn_indices[i], distances[i]);
  }
}

struct VoxelKdTreeAdaptor {
  explicit VoxelKdTreeAdaptor(const GlobalIndexVector& indices) : indices(indices) {}

  inline size_t kdtree_get_point_count() const { return indices.size(); }

  inline int64_t kdtree_get_pt(const size_t idx, const size_t dim) const {
    return indices[idx](dim);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  GlobalIndexVector indices;
};

struct NearestVoxelFinder::Detail {
  using Dist = L2_Simple_Adaptor<int64_t, VoxelKdTreeAdaptor>;
  using KDTree = KDTreeSingleIndexAdaptor<Dist, VoxelKdTreeAdaptor, 3, size_t>;

  explicit Detail(const GlobalIndexVector& indices) : adaptor(indices) {
    kdtree.reset(new KDTree(3, adaptor));
    kdtree->buildIndex();
  }

  ~Detail() = default;

  VoxelKdTreeAdaptor adaptor;
  std::unique_ptr<KDTree> kdtree;
};

NearestVoxelFinder::NearestVoxelFinder(const GlobalIndexVector& indices)
    : internals_(new Detail(indices)) {}

NearestVoxelFinder::~NearestVoxelFinder() {}

void NearestVoxelFinder::find(const GlobalIndex& index,
                              size_t num_to_find,
                              const NearestVoxelFinder::Callback& callback) {
  std::vector<size_t> nn_indices(num_to_find);
  std::vector<int64_t> distances(num_to_find);

  size_t num_found = internals_->kdtree->knnSearch(
      index.data(), num_to_find, nn_indices.data(), distances.data());

  for (size_t i = 0; i < num_found; ++i) {
    callback(internals_->adaptor.indices[nn_indices[i]], nn_indices[i], distances[i]);
  }
}

// TODO(nathan) not working: bug in nanoflann w.r.t accessing removed indices
// during index building
struct GraphDynamicKdTreeAdaptor {
  GraphDynamicKdTreeAdaptor(const SceneGraphLayer& layer) : layer(layer), count(0) {}

  struct AddInfo {
    size_t start;
    size_t num_added = 0;
  };

  struct RemoveInfo {
    size_t index = 0;
    bool valid = false;
  };

  AddInfo addNodes(const std::unordered_set<NodeId>& new_nodes) {
    AddInfo info;
    info.start = count;

    for (const auto& node : new_nodes) {
      if (curr_nodes.count(node)) {
        continue;
      }

      idx_node_map[count] = node;
      node_idx_map[node] = count;
      curr_nodes.insert(node);

      info.num_added++;
      count++;
    }
    LOG(WARNING) << "Added " << info.start << " through " << count - 1;

    return info;
  }

  RemoveInfo removeNode(NodeId node) {
    RemoveInfo info;
    if (!curr_nodes.count(node)) {
      return info;
    }

    info.index = node_idx_map.at(node);
    LOG(WARNING) << "Removing " << info.index;
    node_idx_map.erase(node);
    idx_node_map.erase(info.index);
    curr_nodes.erase(node);
    info.valid = true;
    return info;
  }

  inline size_t kdtree_get_point_count() const { return curr_nodes.size(); }

  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (!idx_node_map.count(idx)) {
      std::stringstream ss;
      auto iter = idx_node_map.begin();
      ss << "[";
      while (iter != idx_node_map.end()) {
        ss << iter->first;
        ++iter;
        if (iter != idx_node_map.end()) {
          ss << ", ";
        }
      }
      ss << "]";

      LOG(FATAL) << "idx: " << idx << " doesn't exist. current count: " << count
                 << " indices: " << ss.str();
    }
    return layer.getPosition(idx_node_map.at(idx))(dim);
  }

  template <class T>
  bool kdtree_get_bbox(T&) const {
    return false;
  }

  const SceneGraphLayer& layer;
  size_t count;
  std::map<size_t, NodeId> idx_node_map;
  std::map<NodeId, size_t> node_idx_map;
  std::unordered_set<NodeId> curr_nodes;
};

FurthestIndexResult findFurthestIndexFromLine(const GlobalIndexVector& indices,
                                              const GlobalIndex& start,
                                              const GlobalIndex& end,
                                              size_t number_source_edges) {
  FurthestIndexResult result;

  GlobalIndexVector line = makeBresenhamLine(start, end);
  if (line.empty()) {
    return result;
  }

  NearestVoxelFinder nearest_voxel_finder(indices);
  for (const auto& line_idx : line) {
    nearest_voxel_finder.find(
        line_idx, 1, [&](const GlobalIndex& index, size_t nn_index, int64_t distance) {
          if (distance > result.distance || !result.valid) {
            result.valid = true;
            result.distance = distance;
            result.from_source = nn_index < number_source_edges;
            result.index = index;
          }
        });
  }

  return result;
}

}  // namespace topology
}  // namespace hydra
