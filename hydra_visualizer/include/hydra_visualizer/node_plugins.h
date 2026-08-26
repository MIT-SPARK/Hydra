#include <ianvs/node_handle.h>

namespace config {
class RosDynamicConfigServer;
}

namespace hydra {

struct NodePlugin {
  virtual ~NodePlugin() = default;
};

class DynamicConfigServer : public NodePlugin {
 public:
  struct Config {};

  DynamicConfigServer(const Config& config, ianvs::NodeHandle nh);

  virtual ~DynamicConfigServer();

 private:
  std::unique_ptr<config::RosDynamicConfigServer> server_;
};

void declare_config(DynamicConfigServer::Config& config);

}  // namespace hydra
