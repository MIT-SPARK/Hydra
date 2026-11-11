#include "hydra_multi/input/file_dsg_input.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include <algorithm>
#include <vector>

namespace hydra_multi {

using config::Path;
using spark_dsg::AgentNodeAttributes;
using spark_dsg::DsgLayers;
using spark_dsg::LayerKey;
using spark_dsg::NodeId;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraph;
using spark_dsg::SceneGraphLayer;

namespace {

static const auto registration_ =
    config::RegistrationWithConfig<Input,
                                   FileDsgInput,
                                   FileDsgInput::Config,
                                   UnitInterfaceState::Ptr,
                                   std::string,
                                   size_t>("FileDsgInput");

void movePartition(SceneGraph& graph,
                   const SceneGraphLayer& prev_layer,
                   char new_char) {
  const auto layer_id = prev_layer.id.layer;
  LOG(INFO) << "Moving Layer " << prev_layer.id << " to "
            << LayerKey(layer_id, new_char);

  // add all nodes
  std::map<NodeId, NodeId> lookup;
  for (const auto& node : prev_layer.nodes()) {
    const NodeSymbol new_id(new_char, NodeSymbol(node.id).categoryId());
    graph.emplaceNode(layer_id, new_id, node.attributes().clone(), new_char);
    lookup[node.id] = new_id;

    auto new_attrs = graph.getNode(new_id).tryAttributes<AgentNodeAttributes>();
    if (!new_attrs) {
      continue;
    }

    new_attrs->external_key =
        NodeSymbol(new_char, NodeSymbol(new_attrs->external_key).categoryId());
  }

  // add all edges
  for (const auto& node : prev_layer.nodes()) {
    const auto new_id = lookup.at(node.id);
    const auto connections = node.connections();
    for (const auto n_id : connections) {
      const auto& prev_edge = graph.getEdge(node.id, n_id);
      const auto iter = lookup.find(n_id);
      const NodeId new_n_id = iter == lookup.end() ? n_id : iter->second;
      graph.insertEdge(new_id, new_n_id, prev_edge.info->clone());
    }
  }

  graph.removeLayer(layer_id, prev_layer.id.partition);
}

// Shifts the agent partitions (one per robot) so that the lowest robot id becomes
// new_id, the same shift the backend applies to the deformation graph robot ids
void shiftRobotIds(size_t new_id, SceneGraph& graph) {
  const auto agent_key = graph.getLayerKey(DsgLayers::AGENTS).value();
  std::vector<size_t> robot_ids;
  for (const auto& layer : graph.layer_partition(agent_key.layer)) {
    const auto iter =
        kimera_pgmo::robot_prefix_to_id.find(static_cast<char>(layer.id.partition));
    if (iter == kimera_pgmo::robot_prefix_to_id.end()) {
      LOG(WARNING) << "Agent layer " << layer.id << " is not a robot partition";
      continue;
    }

    robot_ids.push_back(iter->second);
  }

  if (robot_ids.empty()) {
    LOG(WARNING) << "Could not find agent layer in graph!";
    return;
  }

  std::sort(robot_ids.begin(), robot_ids.end());
  const auto lowest = robot_ids.front();
  if (lowest == new_id) {
    return;
  }

  // move the highest robot first when shifting up (and the lowest first when shifting
  // down) so a partition never moves onto one that is still occupied
  if (new_id > lowest) {
    std::reverse(robot_ids.begin(), robot_ids.end());
  }

  for (const auto robot_id : robot_ids) {
    const auto new_char = kimera_pgmo::GetRobotPrefix(robot_id + new_id - lowest);
    CHECK(robot_id + new_id >= lowest && new_char != '\0')
        << "Robot " << robot_id << " has no valid robot id after shifting the lowest "
        << "robot id " << lowest << " to " << new_id;
    const auto prev_char = kimera_pgmo::robot_id_to_prefix.at(robot_id);
    movePartition(graph, graph.getLayer(agent_key.layer, prev_char), new_char);
  }
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
  state_->dsg_ = SceneGraph::load(config.dsg_json);
  state_->mesh_data_->mesh = state_->dsg_->mesh();
  if (state_->dsg_->mesh()) {
    auto mesh = state_->dsg_->mesh();
    state_->mesh_data_->original.clear();
    state_->mesh_data_->original.reserve(mesh->numVertices());
    for (size_t idx = 0; idx < mesh->numVertices(); ++idx) {
      state_->mesh_data_->original.push_back({mesh->timestamp(idx), mesh->pos(idx)});
    }
  }

  // always shift the agent layers so they match the backend's deformation graph ids
  shiftRobotIds(id_, *state_->dsg_);

  state_->updated = true;
  state_->rebased = true;
}

void FileDsgInput::stop() {}

}  // namespace hydra_multi
