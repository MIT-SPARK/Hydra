#pragma once
#include <hydra/utils/app_plugin.h>

#include <memory>

namespace config {
class RosDynamicConfigServer;
}

namespace hydra {

class ConfigServerPlugin : public AppPlugin {
 public:
  struct Config {};

  explicit ConfigServerPlugin(const Config& config);

  virtual ~ConfigServerPlugin();

 private:
  std::unique_ptr<config::RosDynamicConfigServer> server_;
};

void declare_config(ConfigServerPlugin::Config& config);

}  // namespace hydra
