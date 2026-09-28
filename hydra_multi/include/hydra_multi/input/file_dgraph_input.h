#pragma once
#include <filesystem>

#include "hydra_multi/input/input.h"

namespace hydra_multi {

class FileDGraphInput : public Input {
 public:
  struct Config {
    std::filesystem::path dgrf_path;
    bool include_priors = false;
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
