#include <hydra/utils/logging.h>
#include <hydra_multi/interface/interface_container.h>

namespace hydra_multi {

void InterfaceContainer::insert(UnitInterface::Ptr unit_interface) {
  const auto robot_id = unit_interface->getRobotId();
  interfaces_.emplace(robot_id, std::move(unit_interface));
}

void InterfaceContainer::init() {
  for (const auto& id_interface : interfaces_) {
    id_interface.second->init();
  }
}

void InterfaceContainer::start() {
  for (const auto& id_interface : interfaces_) {
    id_interface.second->start();
  }
}

void InterfaceContainer::stop() {
  for (const auto& id_interface : interfaces_) {
    HYDRA_VLOG(2, "stopping: {}", id_interface.first);
    id_interface.second->stop();
    HYDRA_VLOG(2, "stopped: {}", id_interface.first);
  }
}

void InterfaceContainer::save(const DataDirectory& output) {
  for (const auto& id_interface : interfaces_) {
    HYDRA_VLOG(2, "saving: {}", id_interface.first);
    id_interface.second->save(output);
    HYDRA_VLOG(2, "saved: {}", id_interface.first);
  }
}

InterfaceContainer::StatesPtr InterfaceContainer::states() const {
  StatesPtr states(new States);
  for (const auto& [robot_id, interface] : interfaces_) {
    states->insert({robot_id, interface->getState()});
  }
  return states;
}

}  // namespace hydra_multi
