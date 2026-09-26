#pragma once
#include <memory>

#include "hydra_multi/interface/interface_state.h"

namespace hydra_multi {

class Input {
 public:
  using Ptr = std::shared_ptr<Input>;
  Input(UnitInterfaceState::Ptr state, std::string name, size_t id)
      : state_(state), name_(name), id_(id) {}

  virtual ~Input() = default;

  virtual void init() = 0;

  virtual void stop() = 0;

 protected:
  UnitInterfaceState::Ptr state_;
  std::atomic<bool> should_shutdown_{false};
  std::string name_;
  size_t id_;
};

}  // namespace hydra_multi
