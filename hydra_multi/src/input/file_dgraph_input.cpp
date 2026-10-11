#include "hydra_multi/input/file_dgraph_input.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/path.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/utils/common_functions.h>

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
using kimera_pgmo::DeformationGraph;

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
  auto dgraph = DeformationGraph::load(config.dgrf_path, config.include_priors, id_);
  CHECK(dgraph);

  if (config.fix_as_prior) {
    // TODO(nathan) it would be nice to push this later in the backend
    const auto prefix = kimera_pgmo::robot_id_to_prefix.at(id_);
    const auto poses = dgraph->getTrajectory(prefix);
    std::vector<std::pair<gtsam::Key, gtsam::Pose3>> measurements;
    for (size_t i = 0; i < poses.size(); ++i) {
      const gtsam::Key key = gtsam::Symbol(prefix, i);
      measurements.push_back(std::make_pair(key, poses[i]));
    }

    dgraph->processNodeMeasurements(measurements, config.prior_variance);
  }

  state_->mesh_graph_ = dgraph->getPoseGraph(true, false, false);
  state_->pose_graph_ = dgraph->getPoseGraph(false, true, false);
  state_->updated = true;
  state_->rebased = true;
}

void FileDGraphInput::stop() {}

}  // namespace hydra_multi
