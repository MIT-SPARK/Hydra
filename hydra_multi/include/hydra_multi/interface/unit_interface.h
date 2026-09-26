#pragma once
#include <config_utilities/factory.h>
#include <config_utilities/virtual_config.h>
#include <hydra/backend/dsg_updater.h>
#include <hydra/common/message_queue.h>
#include <hydra/common/shared_dsg_info.h>

#include <atomic>
#include <memory>
#include <string>

#include "hydra_multi/common/dsg_types.h"
#include "hydra_multi/common/types.h"
#include "hydra_multi/input/input.h"
#include "hydra_multi/interface/interface_state.h"
#include "hydra_multi/output/output.h"

namespace hydra_multi {

class UnitInterface {
 public:
  using Ptr = std::unique_ptr<UnitInterface>;

  struct Config : hydra::DsgUpdater::Config {
    size_t robot_id;
    std::string robot_name;
    config::OrderedMap<std::string, config::VirtualConfig<Input, true>> inputs;
    config::OrderedMap<std::string, config::VirtualConfig<Output, true>> outputs;
  } config;

  UnitInterface(const Config& config);
  virtual ~UnitInterface() = default;

  virtual void start() {};

  virtual void init();

  virtual void spin() {};

  virtual void stop();

  virtual void save(const DataDirectory& output);

  void spinOnce();

  size_t getRobotId() const { return config.robot_id; }

  std::string getRobotName() const { return config.robot_name; }

  UnitInterfaceState::Ptr getState() const { return state_; }

 protected:
  // Global frame
  gtsam::Pose3 world_T_robot_;

  // State (operator data)
  UnitInterfaceState::Ptr state_;

  // Inputs
  std::vector<std::pair<std::string, Input::Ptr>> inputs_;

  // Outputs
  std::vector<std::pair<std::string, Output::Ptr>> outputs_;

  // Updated DSG
  hydra::SharedDsgInfo::Ptr merged_dsg_;
  hydra::DsgUpdater::Ptr dsg_updater_;
  hydra::SharedDsgInfo::Ptr updated_dsg_;

  // Shutdown
  std::atomic<bool> should_shutdown_{false};

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<UnitInterface,
                                     UnitInterface,
                                     UnitInterface::Config>("UnitInterface");
};

void declare_config(UnitInterface::Config& config);

}  // namespace hydra_multi
