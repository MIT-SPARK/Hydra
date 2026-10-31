#pragma once
#include <filesystem>

#include "hydra_multi/input/input.h"

namespace hydra_multi {

class FileDGraphInput : public Input {
 public:
  struct Config {
    std::filesystem::path dgrf_path;
    bool include_priors = false;
    //! Keep the robot ids saved in the graph (e.g., a fused multi-robot map) instead of
    //! re-keying everything to this unit's robot id
    bool keep_robot_ids = false;
    //! Fix every loaded trajectory with a prior
    bool fix_as_prior = false;
    double prior_variance = 1.0e-4;
  } const config;

  FileDGraphInput(const Config& config,
                  UnitInterfaceState::Ptr state,
                  std::string name,
                  size_t id);

  ~FileDGraphInput() = default;

  void init() override;

  void stop() override;
};

void declare_config(FileDGraphInput::Config& config);

}  // namespace hydra_multi
