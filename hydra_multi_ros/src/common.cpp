#include "hydra_multi_ros/common.h"

#include <glog/logging.h>
#include <ianvs/node_handle_factory.h>

namespace hydra_multi {

ianvs::NodeHandle getHydraMultiNodeHandle(const std::string& ns) {
  auto nh = ianvs::NodeHandleFactory::getNodeHandle("hydra_multi_ros_node", ns);
  if (!nh) {
    LOG(ERROR) << "'hydra_multi_ros_node' not initialized!";
    throw std::runtime_error("'hydra_multi_ros_node' not initialized");
  }

  return *nh;
}

}  // namespace hydra_multi
