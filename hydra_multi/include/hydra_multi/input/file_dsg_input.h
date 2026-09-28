#pragma once

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
};

void declare_config(FileDsgInput::Config& config);

}  // namespace hydra_multi
