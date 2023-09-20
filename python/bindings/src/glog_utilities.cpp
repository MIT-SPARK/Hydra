#include "hydra/bindings/glog_utilities.h"

#include <glog/logging.h>

#include <iostream>

namespace hydra::python {

std::unique_ptr<GlogSingleton> GlogSingleton::instance_;

GlogSingleton& GlogSingleton::instance() {
  if (!instance_) {
    instance_.reset(new GlogSingleton());
  }
  return *instance_;
}

void GlogSingleton::setLogLevel(int log_level, int verbosity, bool override_settings) {
  if (configured_ && !override_settings) {
    return;
  }

  FLAGS_minloglevel = log_level;
  FLAGS_v = verbosity;
  configured_ = true;
}

void GlogSingleton::setLogDirectory(const std::string& dirname) {
  std::cerr << "[Hydra Python] Logging to: " << dirname << std::endl;
  FLAGS_log_dir = dirname;
  FLAGS_logtostderr = false;
  FLAGS_alsologtostderr = false;
  FLAGS_stderrthreshold = 1;
  FLAGS_minloglevel = 0;
  FLAGS_v = 5;
}

GlogSingleton::GlogSingleton() {
  FLAGS_logtostderr = true;
  google::InitGoogleLogging("hydra_python");
  google::InstallFailureSignalHandler();
}

namespace glog_utilities {

using namespace pybind11::literals;

void addBindings(pybind11::module_& m) {
  m.def(
      "set_glog_level",
      [](int glog_level, int verbosity, bool override_settings) {
        GlogSingleton::instance().setLogLevel(glog_level, verbosity, override_settings);
      },
      "log_level"_a,
      "verbosity"_a = 0,
      "override_settings"_a = true);
  m.def(
      "set_glog_dir",
      [](const std::string& dirname) {
        GlogSingleton::instance().setLogDirectory(dirname);
      },
      "dirname"_a);
}

}  // namespace glog_utilities

}  // namespace hydra::python
