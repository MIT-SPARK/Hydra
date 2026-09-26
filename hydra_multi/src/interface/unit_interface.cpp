#include "hydra_multi/interface/unit_interface.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/utils/pgmo_mesh_traits.h>
#include <hydra_multi/input/file_dgraph_input.h>
#include <hydra_multi/input/file_dsg_input.h>
#include <spark_dsg/serialization/graph_binary_serialization.h>

#include "hydra_multi/common/multi_global_info.h"

namespace hydra_multi {

void declare_config(UnitInterface::Config& config) {
  using namespace config;
  base<hydra::DsgUpdater::Config>(config);
  name("UnitInterface::Config");
  // ! Robot ID
  field(config.robot_id, "robot_id");

  // ! Robot ID
  field(config.robot_name, "robot_name");

  // ! Input entities
  field(config.inputs, "inputs");

  // ! Output entities
  field(config.outputs, "outputs");
}

UnitInterface::UnitInterface(const Config& config)
    : config(config::checkValid(config)), state_(new UnitInterfaceState) {
  for (const auto& [name, input] : config.inputs) {
    inputs_.emplace_back(name,
                         input.create(state_, config.robot_name, config.robot_id));
  }

  for (const auto& [name, output] : config.outputs) {
    outputs_.emplace_back(name, output.create(state_));
  }

  merged_dsg_ = MultiGlobalInfo::instance().createSharedDsg();
  dsg_updater_.reset(new hydra::DsgUpdater(config, state_->dsg_, merged_dsg_));
}

void UnitInterface::init() {
  LOG(INFO) << " [Robot " << config.robot_id << " (" << config.robot_name
            << ") unit interface] created!";
  for (const auto& [input_name, input] : inputs_) {
    VLOG(3) << "Initializing input: " << input_name;
    CHECK(input) << "Invalid input: '" << input_name << "'";
    input->init();
  }

  for (const auto& [output_name, output] : outputs_) {
    VLOG(3) << "Initializing output: " << output_name;
    CHECK(output) << "Invalid input: '" << output_name << "'";
    output->init();
  }
}

void UnitInterface::spinOnce() {
  if (state_->rebased) {
    if (dsg_updater_) {
      // Update DSG
    }
  }

  for (const auto& [output_name, output] : outputs_) {
    VLOG(3) << "Triggering output: " << output_name;
    output->trigger();
  }
}

void UnitInterface::stop() {
  VLOG(3) << "Shutting down interface for " << config.robot_name;
  should_shutdown_ = true;
  for (const auto& [input_name, input] : inputs_) {
    VLOG(3) << "Shutting down input: " << input_name;
    input->stop();
  }

  for (const auto& [output_name, output] : outputs_) {
    VLOG(3) << "Shutting down output: " << output_name;
    output->stop();
  }
  VLOG(3) << config.robot_name << " interface stopped";
}

void UnitInterface::save(const DataDirectory& output) {
  const auto path = output.path(std::format("interfaces/robot_{}", config.robot_id));
  state_->save(path);
}

}  // namespace hydra_multi
