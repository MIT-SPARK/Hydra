#pragma once
#include <config_utilities/config_utilities.h>

#include "hydra_multi/input/input.h"

namespace hydra_multi {

class FileDsgInput : public Input {
 public:
  struct Config {
    std::string dsg_json;
    bool force_robot_id = false;
  } const config;

  FileDsgInput(const Config& config,
               UnitInterfaceState::Ptr state,
               std::string name,
               size_t id);

  ~FileDsgInput() = default;

  void init() override;

  void stop() override;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<Input,
                                     FileDsgInput,
                                     FileDsgInput::Config,
                                     UnitInterfaceState::Ptr,
                                     std::string,
                                     size_t>("FileDsgInput");
};
void declare_config(FileDsgInput::Config& config);
}  // namespace hydra_multi
