#include "hydra_multi/operators/dynamic_scene_graph_operator.h"

#include <spark_dsg/serialization/graph_binary_serialization.h>

#include <queue>

#include "hydra_multi/interface/utils.h"
namespace hydra_multi {
using GraphMergeConfig = spark_dsg::GraphMergeConfig;
bool DynamicSceneGraphOperator::incrementalAppend(
    const SceneGraphDelta& incremental_source) {
  // Appending scene graph nodes and edges from source to data_
  if (data_->empty()) {
    auto new_graph = spark_dsg::io::binary::readGraph(incremental_source);
    data_->reset(new_graph->layer_keys());
  }
  auto data_clone = data_->clone();
  spark_dsg::io::binary::updateGraph(*data_clone, incremental_source);
  data_->mergeGraph(*data_clone, {}, &append_T_data_);
  // TODO(Yun) this doesn't currently append properly (with the transform)
  return true;
}

bool DynamicSceneGraphOperator::update(const DynamicSceneGraph& source) {
  // Updating the scene graph in data_ according to source
  // Returns an error if data_ node does not exist in source
  if (source.numNodes() < data_->numNodes()) {
    return false;
    // TODO(Yun) not a comprehensive check
  }

  // Update transform
  updateAppendTransform(source);

  // TODO(nathan) think about whether this should be by key
  std::map<LayerId, bool> layer_updates;
  for (const auto& layer_id : data_->layer_ids()) {
    layer_updates.insert({layer_id, true});
  }
  GraphMergeConfig config{nullptr, &layer_updates, true, true};
  return data_->mergeGraph(source, config);
}

bool DynamicSceneGraphOperator::rebase(const DynamicSceneGraph& source) {
  // Rebase by first adding and updating from source, then figure out transform to add
  // parts of data_ that does not exist in source

  // Update transform
  updateAppendTransform(source);

  auto orig = data_->clone();

  // Remove the non overlapping nodes
  std::vector<spark_dsg::NodeId> non_overlapping;
  for (const auto& node_layer : data_->node_lookup()) {
    if (!source.hasNode(node_layer.first)) {
      non_overlapping.push_back(node_layer.first);
    }
  }

  for (const auto& non_overlap_node : non_overlapping) {
    data_->removeNode(non_overlap_node);
  }

  // TODO(nathan) think about whether this should be by key
  std::map<LayerId, bool> layer_updates;
  for (const auto& layer_id : data_->layer_ids()) {
    layer_updates.insert({layer_id, true});
  }
  GraphMergeConfig config{nullptr, &layer_updates, true, true};
  if (!data_->mergeGraph(source, config)) {
    return false;
  }

  // Get the nodes in data_ not in source and update
  return data_->mergeGraph(*orig, {}, &append_T_data_);
}

bool DynamicSceneGraphOperator::merge(const DynamicSceneGraph& source) {
  // To merge find the new parts of source and append
  // Don't need to update append_T but need to compute source_T_data
  if (data_->empty()) {
    data_->reset(source.layer_keys());
  }
  Eigen::Isometry3d source_T_data = computeSourceDataTransform(source);
  return data_->mergeGraph(source, {}, &source_T_data);
}

Eigen::Isometry3d DynamicSceneGraphOperator::computeSourceDataTransform(
    const DynamicSceneGraph& source) {
  // Find transform from original to scene graph
  // Align with the agent nodes (or other layers?)
  // TODO(Yun) make this a config
  size_t num_pts_for_align = 9;
  std::queue<Eigen::Vector3d> source_pts_queue;
  std::queue<Eigen::Vector3d> pts_queue;
  const auto& node_lookup = data_->node_lookup();
  for (auto node_layer = node_lookup.rbegin(); node_layer != node_lookup.rend();
       ++node_layer) {
    if (!source.hasNode(node_layer->first)) {
      continue;
    }

    pts_queue.push(data_->getNode(node_layer->first).attributes().position);
    source_pts_queue.push(source.getNode(node_layer->first).attributes().position);
    if (source_pts_queue.size() >= num_pts_for_align) {
      break;
    }
  }

  std::vector<Eigen::Vector3d> source_pts;
  std::vector<Eigen::Vector3d> pts;
  for (size_t i = 0; i < num_pts_for_align; i++) {
    source_pts.push_back(source_pts_queue.front());
    pts.push_back(pts_queue.front());
    source_pts_queue.pop();
    pts_queue.pop();
  }

  Eigen::Isometry3d source_T_data;
  estimateRigidTransformSVD(source_pts, pts, source_T_data);
  return source_T_data;
}

void DynamicSceneGraphOperator::updateAppendTransform(const DynamicSceneGraph& source) {
  append_T_data_ = computeSourceDataTransform(source).inverse();
}

}  // namespace hydra_multi
