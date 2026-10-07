#include "hydra_multi/input/file_dgraph_input.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <spark_dsg/node_attributes.h>

namespace hydra_multi {
namespace {

static const auto registration_ =
    config::RegistrationWithConfig<Input,
                                   FileDGraphInput,
                                   FileDGraphInput::Config,
                                   UnitInterfaceState::Ptr,
                                   std::string,
                                   size_t>("FileDGraphInput");

}

using config::Path;
using spark_dsg::AgentNodeAttributes;
using spark_dsg::DsgLayers;

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
    size_t num_layers = 0;
    const auto agent_layer_id = state_->dsg_->getLayerKey(DsgLayers::AGENTS)->layer;
    for (const auto& layer : state_->dsg_->layer_partition(agent_layer_id)) {
      // skip non-robot partitions (e.g., sub-keyframes)
      if (!kimera_pgmo::robot_prefix_to_id.count(layer.id.partition)) {
        continue;
      }

      ++num_layers;
      if (num_layers > 1) {
        LOG(FATAL) << "FileUnitInterface assumes loading a single robot";
      }

      LOG(INFO) << "Found agent layer with " << layer.numNodes() << " poses";
      for (const auto& node : layer.nodes()) {
        const auto attrs = node.tryAttributes<AgentNodeAttributes>();
        if (attrs) {
          stamps[id_].push_back(attrs->timestamp.count());
        }
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
