#pragma once
#include <hydra/utils/app_plugin.h>

#include <memory>
#include <thread>

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

class SpinPlugin : public AppPlugin {
 public:
  struct Config {
    bool multi_thread = true;
    double spin_rate = 50.0;
  } const config;

  explicit SpinPlugin(const Config& config);

  ~SpinPlugin();

 private:
  void spin();

  std::atomic<bool> should_shutdown_;
  std::thread spin_thread_;
};

void declare_config(SpinPlugin::Config& config);

}  // namespace hydra
