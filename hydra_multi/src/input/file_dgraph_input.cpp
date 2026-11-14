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
  field(config.include_mesh, "include_mesh");

  check<Path::Exists>(config.dgrf_path, "dgrf_path");
  check(config.prior_variance, GT, 0.0, "prior_variance");
}

FileDGraphInput::FileDGraphInput(const Config& config,
                                 UnitInterfaceState::Ptr state,
                                 std::string name,
                                 size_t id)
    : Input(state, name, id), config(config) {}

void FileDGraphInput::init() {
  // keep the saved robot ids: the backend shifts them so the lowest is the unit id
  auto dgraph = DeformationGraph::load(config.dgrf_path, config.include_priors);
  CHECK(dgraph);

  if (config.fix_as_prior) {
    dgraph->fixPosesAsPriors(config.prior_variance);
  }

  if (config.include_mesh) {
    state_->mesh_graph_ = dgraph->getPoseGraph(true, false, false);
  }

  state_->pose_graph_ = dgraph->getPoseGraph(false, true, false);
  state_->updated = true;
  state_->rebased = true;
}

void FileDGraphInput::stop() {}

}  // namespace hydra_multi
