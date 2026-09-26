#include "hydra_multi/input/file_dgraph_input.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>

namespace hydra_multi {

using config::Path;

void declare_config(FileDGraphInput::Config& config) {
  using namespace config;
  name("FileDGraphInput::Config");
  field<Path>(config.dgrf_path, "dgrf_path");
  field(config.include_priors, "include_priors");
  field(config.fix_as_prior, "fix_as_prior");
  field(config.prior_variance, "prior_variance");

  check<Path::Exists>(config.dgrf_path, "dgrf_path");
  check(config.prior_variance, GT, 0.0, "prior_variance");
}

FileDGraphInput::FileDGraphInput(const Config& config,
                                 UnitInterfaceState::Ptr state,
                                 std::string name,
                                 size_t id)
    : Input(state, name, id), config(config) {}

void FileDGraphInput::init() {
  kimera_pgmo::DeformationGraph dgraph;
  dgraph.load(config.dgrf_path, true, true, id_, config.include_priors);

  std::map<size_t, std::vector<Timestamp>> stamps;
  stamps[id_] = Timestamps();
  std::lock_guard<std::mutex> state_lock(state_->mutex);
  if (state_->dsg_) {
    const auto agent_layer_id = state_->dsg_->getLayerKey(DsgLayers::AGENTS)->layer;
    if (state_->dsg_->layer_partition(agent_layer_id).size() > 1) {
      LOG(FATAL) << "FileUnitInterface assumes loading a single robot";
    }

    for (const auto& [prefix, layer] : state_->dsg_->layer_partition(agent_layer_id)) {
      LOG(INFO) << "Found agent layer with " << layer->numNodes() << " poses";
      for (const auto& [node_id, node] : layer->nodes()) {
        stamps[id_].push_back(
            node->attributes<AgentNodeAttributes>().timestamp.count());
      }
    }
  } else {
    LOG(ERROR) << "FileUnitInterface for '" << name_
               << "' missing scene graph in state when loading timestamps";
  }

  if (config.fix_as_prior) {
    // TODO(nathan) it would be nice to push this later in the backend
    const auto prefix = kimera_pgmo::robot_id_to_prefix.at(id_);
    const auto poses = dgraph.getTrajectory(prefix);
    std::vector<std::pair<gtsam::Key, gtsam::Pose3>> measurements;
    for (size_t i = 0; i < poses.size(); ++i) {
      const gtsam::Key key = gtsam::Symbol(prefix, i);
      measurements.push_back(std::make_pair(key, poses[i]));
    }

    dgraph.processNodeMeasurements(measurements, config.prior_variance);
  }

  state_->mesh_graph_ = dgraph.getPoseGraph(stamps, true, false, false);
  state_->pose_graph_ = dgraph.getPoseGraph(stamps, false, true, false);
  state_->updated = true;
  state_->rebased = true;
}

void FileDGraphInput::stop() {}

}  // namespace hydra_multi
