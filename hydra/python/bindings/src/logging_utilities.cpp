#include "hydra/bindings/logging_utilities.h"

#include <pybind11/stl/filesystem.h>

#include "hydra/utils/logging.h"

namespace hydra::python::logging_utilities {

void addBindings(pybind11::module_& m) {
  using namespace pybind11::literals;
  m.def("set_log_to_stderr", &hydra::logging::setLogToStderr);
  m.def("set_log_level", &hydra::logging::setLogLevel, "log_level"_a);
  m.def(
      "set_log_directory",
      [](const std::filesystem::path& directory, const std::string& stderr_level) {
        hydra::logging::setLogDirectory(directory.string(), stderr_level);
      },
      "directory"_a,
      "stderr_level"_a = "error");
}

}  // namespace hydra::python::logging_utilities
