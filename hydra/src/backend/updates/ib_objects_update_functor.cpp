#include "hydra/backend/updates/ib_objects_update_functor.h"

#include <config_utilities/config.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/graph_utilities.h>
#include <spark_dsg/printing.h>

#include "hydra/utils/nearest_neighbor_utilities.h"
#include "hydra/utils/probability_utilities.h"
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

static const auto functor_reg =
    config::RegistrationWithConfig<UpdateFunctor,
                                   IBObjectsUpdateFunctor,
                                   IBObjectsUpdateFunctor::Config>("ObjectsIBFunctor");

void mergeObjectAttributes(const KhronosObjectAttributes& from,
                           KhronosObjectAttributes& into) {
  // Update the bounding box. Do this first as it is the reference for the mesh.
  BoundingBox new_box = into.bounding_box;
  new_box.merge(from.bounding_box);

  // Adjust the old vertex positions.
  for (auto& vertex : into.mesh.points) {
    vertex = new_box.pointToBoxFrame(into.bounding_box.pointToWorldFrame(vertex));
  }

  // Merge incoming vertices.
  const size_t num_previous_vertices = into.mesh.numVertices();
  into.mesh.resizeVertices(num_previous_vertices + from.mesh.numVertices());
  for (size_t i = 0; i < from.mesh.numVertices(); ++i) {
    into.mesh.setPos(
        num_previous_vertices + i,
        new_box.pointToBoxFrame(from.bounding_box.pointToWorldFrame(from.mesh.pos(i))));
    into.mesh.setColor(num_previous_vertices + i, from.mesh.color(i));
  }

  // Merges incoming faces.
  const size_t idx_offset = into.mesh.numVertices();
  into.mesh.faces.reserve(from.mesh.faces.size() + into.mesh.faces.size());
  for (const auto& face : from.mesh.faces) {
    spark_dsg::Mesh::Face new_face = face;
    for (size_t i = 0; i < new_face.size(); ++i) {
      new_face[i] += idx_offset;
    }
    into.mesh.faces.emplace_back(new_face);
  }

  into.bounding_box = new_box;
}

bool isNodeActive(const SceneGraphNode& node,
                  const std::map<NodeId, size_t>& node_to_component,
                  const std::set<NodeId>& invalid) {
  return !node_to_component.count(node.id) && !invalid.count(node.id);
}

NodeAttributes::Ptr getMergedAttributes(const SceneGraph& graph,
                                        const std::vector<NodeId>& nodes) {
  if (nodes.empty()) {
    return nullptr;
  }

  auto iter = nodes.begin();
  CHECK(graph.hasNode(*iter));
  const auto& node = graph.getNode(*iter);
  ++iter;

  auto attrs_ptr = node.attributes().clone();
  auto& attrs = *CHECK_NOTNULL(dynamic_cast<KhronosObjectAttributes*>(attrs_ptr.get()));
  attrs.semantic_feature = attrs.semantic_feature.rowwise().mean().eval();

  while (iter != nodes.end()) {
    const auto& other = graph.getNode(*iter);
    const auto& other_attrs = other.attributes<KhronosObjectAttributes>();
    attrs.position += other_attrs.position;
    attrs.semantic_feature += other_attrs.semantic_feature.rowwise().mean();
    // TODO(nathan) update khronos to add the attribute merging somewhere convenient
    mergeObjectAttributes(other_attrs, attrs);
    ++iter;
  }

  attrs.position /= nodes.size();
  attrs.semantic_feature /= nodes.size();
  return attrs_ptr;
}

std::optional<std::pair<NodeId, bool>> getBestParent(const SceneGraph& graph,
                                                     const std::vector<NodeId>& nodes) {
  std::vector<NodeId> active;
  std::vector<NodeId> archived;
  for (const auto node_id : nodes) {
    const auto& node = graph.getNode(node_id);
    const auto parent = node.getParent();
    if (!parent) {
      continue;
    }

    const auto& parent_node = graph.getNode(*parent);
    if (parent_node.attributes().is_active) {
      active.push_back(*parent);
    } else {
      archived.push_back(*parent);
    }
  }

  if (!archived.empty()) {
    return std::make_pair(archived.front(), false);
  }

  if (!active.empty()) {
    return std::make_pair(active.front(), true);
  }

  return std::nullopt;
}

FeatureMap<NodeId> getLayerEmbeddings(const SceneGraphLayer& layer,
                                      const std::vector<NodeId>& nodes) {
  FeatureMap<NodeId> features;
  for (const auto node_id : nodes) {
    auto node = layer.findNode(node_id);
    if (!node) {
      LOG(ERROR) << "Node " << NodeSymbol(node_id).str() << " not found!";
      continue;
    }

    auto attrs = node->tryAttributes<SemanticNodeAttributes>();
    if (!attrs) {
      LOG(ERROR) << "Node " << NodeSymbol(node_id).str() << " has invalid attributes!";
      continue;
    }

    features[node_id] = attrs->semantic_feature.rowwise().mean();
  }

  return features;
}

std::vector<FeatureVector> getAllFeatures(const SceneGraphLayer& layer) {
  std::vector<FeatureVector> features;
  for (const auto& [node_id, node] : layer.nodes()) {
    auto attrs = node->tryAttributes<SemanticNodeAttributes>();
    if (!attrs) {
      LOG(ERROR) << "Node " << NodeSymbol(node_id).str() << " has invalid attributes!";
      continue;
    }

    features.push_back(attrs->semantic_feature.rowwise().mean());
  }

  return features;
}

}  // namespace

using timing::ScopedTimer;

void declare_config(IBObjectsUpdateFunctor::Config& config) {
  using namespace config;
  name("IBObjectsUpdateFunctor::Config");
  base<VerbosityConfig>(config);
  field<CharConversion>(config.id_prefix, "id_prefix");
  field(config.source_layer, "source_layer");
  field(config.target_layer, "target_layer");
  field(config.parent_layer, "parent_layer");
  field(config.min_segment_score, "min_segment_score");
  field(config.min_object_score, "min_object_score");
  field(config.tasks, "tasks");
  field(config.clustering, "selector");
  config.edge_checker.setOptional();
  field(config.edge_checker, "edge_checker");
  config.metric.setOptional();
  field(config.metric, "metric");
}

ComponentInfo::ComponentInfo(const Config& config,
                             const EmbeddingGroup& tasks,
                             const EmbeddingDistance& metric,
                             const SceneGraphLayer& layer,
                             const std::vector<NodeId>& nodes,
                             double I_xy_full)
    : ws(config, layer.edges(), getLayerEmbeddings(layer, nodes), tasks, metric),
      segments(nodes) {
  ws.reweight(I_xy_full, static_cast<double>(nodes.size()) / layer.numNodes());
  AgglomerativeClustering::cluster(ws);
}

IBObjectsUpdateFunctor::IBObjectsUpdateFunctor(const Config& config)
    : config(config::checkValid(config)),
      tasks_(config.tasks.create()),
      metric_(config.metric.create()),
      edge_checker_(config.edge_checker.create()),
      next_node_id_(config.id_prefix, 0) {}

void IBObjectsUpdateFunctor::call(const SceneGraph&,
                                  SharedDsgInfo& dsg,
                                  const UpdateInfo::ConstPtr& info) const {
  ScopedTimer timer("backend/object_clustering", info->timestamp_ns);
  auto& graph = *dsg.graph;

  // repair broken edges between objects and places
  updateActiveParents(graph);
  // detect edges between segments (and active connected components)
  const auto active_components = addSegmentEdges(graph);
  // remove all previous components that are active
  clearActiveComponents(graph, active_components);
  // construct new components and cluster into objects
  detectObjects(graph);
}

void IBObjectsUpdateFunctor::clearActiveComponents(
    SceneGraph& graph, const std::set<size_t>& active) const {
  auto iter = components_.begin();
  while (iter != components_.end()) {
    if (!active.count(iter->first)) {
      ++iter;
      continue;
    }

    for (const auto node_id : iter->second->segments) {
      node_to_component_.erase(node_id);
    }

    for (const auto& node_id : iter->second->objects) {
      graph.removeNode(node_id);
      active_.erase(node_id);
    }

    components_ids_.markFree(iter->first);
    iter = components_.erase(iter);
  }
}

std::set<size_t> IBObjectsUpdateFunctor::addSegmentEdges(SceneGraph& graph) const {
  const auto& segments = graph.getLayer(config.source_layer);

  std::set<size_t> active_components;
  for (const auto& [node_id, node] : segments.nodes()) {
    if (ignored_.count(node_id)) {
      continue;
    }

    if (node_to_component_.count(node_id)) {
      // only examine new nodes
      continue;
    }

    auto& attrs = node->attributes<SemanticNodeAttributes>();
    const Eigen::VectorXf feature = attrs.semantic_feature.rowwise().mean();
    const auto result = tasks_->getBestScore(*metric_, feature);
    if (result.score < config.min_segment_score) {
      MLOG(1) << "Skipping segment with score: " << result.score;
      ignored_.insert(node_id);
      attrs.is_active = false;
      continue;
    }

    for (const auto& [other_id, other_node] : segments.nodes()) {
      if (other_id == node_id) {
        continue;
      }

      if (!edge_checker_->match(attrs, other_node->attributes())) {
        continue;
      }

      graph.insertEdge(node_id, other_id);
      const auto iter = node_to_component_.find(other_id);
      if (iter != node_to_component_.end()) {
        active_components.insert(iter->second);
      }
    }
  }

  return active_components;
}

void IBObjectsUpdateFunctor::detectObjects(SceneGraph& graph) const {
  const auto& segments = graph.getLayer(config.source_layer);

  const auto M = tasks_->size() + 1;
  const auto all_features = getAllFeatures(segments);
  const auto N = all_features.size();

  const auto px_all = Eigen::VectorXd::Constant(N, 1.0 / static_cast<double>(N));
  const auto py_all = Eigen::VectorXd::Constant(M, 1.0 / static_cast<double>(M));
  const auto py_x_all = AgglomerativeClustering::Workspace::compute_py_x(
      config.clustering, all_features, *tasks_, *metric_);

  const auto I_xy_all = mutualInformation(py_all, px_all, py_x_all);

  // connected component search
  const auto new_components = graph_utilities::getConnectedComponents(
      segments,
      [&](const auto& n) { return isNodeActive(n, node_to_component_, ignored_); },
      [&](const auto& edge) {
        const auto source_active =
            isNodeActive(segments.getNode(edge.source), node_to_component_, ignored_);
        const auto target_active =
            isNodeActive(segments.getNode(edge.target), node_to_component_, ignored_);
        return source_active && target_active;
      });

  // reassign components
  for (const auto& nodes : new_components) {
    size_t new_id = components_ids_.next();
    auto new_component = std::make_unique<ComponentInfo>(
        config.clustering, *tasks_, *metric_, segments, nodes, I_xy_all);
    for (const auto node_id : nodes) {
      node_to_component_[node_id] = new_id;
    }

    const auto clusters = new_component->ws.getClusters();
    for (const auto& cluster : clusters) {
      VLOG(5) << "Cluster: " << displayNodeSymbolContainer(cluster);

      auto attrs = getMergedAttributes(graph, cluster);
      if (!attrs) {
        LOG(ERROR) << "empty cluster!";
        continue;
      }

      const auto& feature =
          CHECK_NOTNULL(dynamic_cast<SemanticNodeAttributes*>(attrs.get()))
              ->semantic_feature;
      const auto result = tasks_->getBestScore(*metric_, feature);
      if (result.score < config.min_object_score) {
        VLOG(1) << "Skipping object with score: " << result.score;
        continue;
      }

      graph.emplaceNode(config.target_layer, next_node_id_, std::move(attrs));
      new_component->objects.push_back(next_node_id_);

      const auto parent = getBestParent(graph, cluster);
      if (!parent) {
        LOG(WARNING) << "object '" << next_node_id_.str() << "' without parent!";
        active_.insert(next_node_id_);
      } else {
        auto&& [parent_id, parent_active] = *parent;
        graph.insertEdge(next_node_id_, parent_id);
        if (parent_active) {
          active_.insert(next_node_id_);
        }
      }

      ++next_node_id_;
    }

    components_.emplace(new_id, std::move(new_component));
  }
}

void IBObjectsUpdateFunctor::updateActiveParents(SceneGraph& graph) const {
  std::vector<NodeId> place_ids;
  const auto& places = graph.getLayer(config.parent_layer);
  for (const auto& [node_id, node] : places.nodes()) {
    place_ids.push_back(node_id);
  }

  for (const auto& [node_id, node] : graph.getLayer(config.target_layer).nodes()) {
    active_.insert(node_id);
  }

  NearestNodeFinder places_finder(places, place_ids);

  auto iter = active_.begin();
  while (iter != active_.end()) {
    const auto& node = graph.getNode(*iter);
    const auto parent_id = node.getParent();
    if (parent_id) {
      const bool is_active = places.getNode(*parent_id).attributes().is_active;
      if (is_active) {
        ++iter;
      } else {
        iter = active_.erase(iter);
      }

      continue;
    }

    bool found = false;
    const auto pos = node.attributes().position;
    places_finder.find(pos, 1, false, [&](NodeId place_id, size_t, double) {
      graph.insertEdge(place_id, node.id);
      found = true;
    });

    if (found) {
      iter = active_.erase(iter);
    } else {
      ++iter;
    }
  }
}

}  // namespace hydra
