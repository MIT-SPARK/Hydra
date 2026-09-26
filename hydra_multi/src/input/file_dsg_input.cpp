#include "hydra_multi/input/file_dsg_input.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/printing.h>

namespace hydra_multi {

using config::Path;

namespace {

inline void rewireRobotId(int new_id, DynamicSceneGraph& graph) {
  const auto agent_key = graph.getLayerKey(DsgLayers::AGENTS).value();
  const SceneGraphLayer* prev_layer = nullptr;
  for (const auto& [prefix, layer] : graph.layer_partition(agent_key.layer)) {
    prev_layer = layer.get();
    break;
  }

  if (!prev_layer) {
    LOG(WARNING) << "Could not find agent layer in graph!";
    return;
  }

  const auto new_char = kimera_pgmo::GetRobotPrefix(new_id);
  if (prev_layer->id.partition == static_cast<uint32_t>(new_char)) {
    return;
  }

  LOG(INFO) << "Moving Layer " << prev_layer->id << " to "
            << LayerKey(agent_key.layer, new_char);

  // add all nodes
  std::map<NodeId, NodeId> lookup;
  for (const auto& [node_id, node] : prev_layer->nodes()) {
    const NodeSymbol new_id(new_char, NodeSymbol(node_id).categoryId());
    graph.emplaceNode(agent_key.layer, new_id, node->attributes().clone(), new_char);
    lookup[node_id] = new_id;

    auto new_attrs = graph.getNode(new_id).tryAttributes<AgentNodeAttributes>();
    if (!new_attrs) {
      continue;
    }

    new_attrs->external_key =
        NodeSymbol(new_char, NodeSymbol(new_attrs->external_key).categoryId());
  }

  // add all edges
  for (const auto& [node_id, node] : prev_layer->nodes()) {
    const auto new_id = lookup.at(node_id);
    const auto connections = node->connections();
    for (const auto n_id : connections) {
      const auto& prev_edge = graph.getEdge(node_id, n_id);
      const auto iter = lookup.find(n_id);
      const NodeId new_n_id = iter == lookup.end() ? n_id : iter->second;
      graph.insertEdge(new_id, new_n_id, prev_edge.info->clone());
    }
  }

  graph.removeLayer(prev_layer->id.layer, prev_layer->id.partition);
}

}  // namespace

void declare_config(FileDsgInput::Config& config) {
  using namespace config;
  name("FileDsgInput::Config");
  field<Path>(config.dsg_json, "dsg_json");
  field(config.force_robot_id, "force_robot_id");

  check<Path::Exists>(config.dsg_json, "dsg_json");
}

FileDsgInput::FileDsgInput(const Config& config,
                           UnitInterfaceState::Ptr state,
                           std::string name,
                           size_t id)
    : Input(state, name, id), config(config) {}

void FileDsgInput::init() {
  std::lock_guard<std::mutex> state_lock(state_->mutex);
  // Load DSG and Mesh
  state_->dsg_ = DynamicSceneGraph::load(config.dsg_json);
  state_->mesh_data_->mesh = state_->dsg_->mesh();
  if (state_->dsg_->mesh()) {
    auto mesh = state_->dsg_->mesh();
    state_->mesh_data_->original_vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);
    state_->mesh_data_->vertex_stamps.reset(new Timestamps);

    for (size_t idx = 0; idx < mesh->numVertices(); ++idx) {
      auto vertex_pos = mesh->pos(idx);
      state_->mesh_data_->original_vertices->push_back(
          {vertex_pos.x(), vertex_pos.y(), vertex_pos.z()});
      state_->mesh_data_->vertex_stamps->push_back(mesh->timestamp(idx));
    }
  }

  if (config.force_robot_id) {
    rewireRobotId(id_, *state_->dsg_);
  }

  state_->updated = true;
  state_->rebased = true;
}

void FileDsgInput::stop() {}

}  // namespace hydra_multi
