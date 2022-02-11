#include "hydra_utils/display_utils.h"

namespace hydra_utils {

std::string getHumanReadableMemoryString(size_t bytes) {
  const size_t log_bytes = static_cast<size_t>(std::floor(std::log2(bytes)));
  const size_t unit_index = log_bytes / 10;

  std::stringstream ss;
  ss << std::setprecision(3);
  if (unit_index == 0) {
    ss << bytes << " bytes";
  } else if (unit_index == 1) {
    ss << (static_cast<double>(bytes) / std::pow(2.0, 10)) << " KiB";
  } else if (unit_index == 2) {
    ss << (static_cast<double>(bytes) / std::pow(2.0, 20)) << " MiB";
  } else {
    ss << (static_cast<double>(bytes) / std::pow(2.0, 30)) << " GiB";
  }

  return ss.str();
}

}  // namespace hydra_utils
