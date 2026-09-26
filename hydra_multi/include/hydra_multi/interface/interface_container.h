#pragma once
#include <map>

#include "hydra_multi/interface/unit_interface.h"

namespace hydra_multi {

class InterfaceContainer {
 public:
  using States = std::map<size_t, UnitInterfaceState::Ptr>;
  using StatesPtr = std::shared_ptr<States>;
  using Ptr = std::shared_ptr<InterfaceContainer>;

  void init();

  void start();

  void stop();

  void save(const DataDirectory& output);

  void insert(UnitInterface::Ptr unit_interface);

  StatesPtr states() const;

 private:
  std::map<size_t, UnitInterface::Ptr> interfaces_;
};

}  // namespace hydra_multi
