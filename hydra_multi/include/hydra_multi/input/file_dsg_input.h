#pragma once

#include "hydra_multi/input/input.h"

namespace hydra_multi {

class FileDsgInput : public Input {
 public:
  struct Config {
    std::string dsg_json;
    //! Deprecated and ignored: the robot ids of the agent layers are always shifted so
    //! the lowest one is this unit's robot id (e.g., 0 -> 1 for a single map, 1..N
    //! unchanged for a fused map), matching the backend's deformation graph ids
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
