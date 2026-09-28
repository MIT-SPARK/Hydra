#pragma once
#include <memory>

#include "hydra_multi/interface/interface_state.h"

namespace hydra_multi {

class Output {
 public:
  using Ptr = std::shared_ptr<Output>;
  Output(UnitInterfaceState::Ptr state) : state_(state) {}
  virtual ~Output() = default;

  virtual void init() = 0;
  virtual void stop() = 0;
  virtual void trigger() = 0;

 protected:
  UnitInterfaceState::Ptr state_;
};
}  // namespace hydra_multi
