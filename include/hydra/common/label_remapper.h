#pragma once

#include <config_utilities/config_utilities.h>

#include <map>

namespace hydra {

struct LabelRemapRow {
  uint32_t sub_id;
  uint32_t super_id;
};
void declare_config(LabelRemapRow& config);

class LabelRemapper {
 public:
  // Construction
  LabelRemapper();
  LabelRemapper(const std::string& remapping_file);
  virtual ~LabelRemapper() = default;

  std::optional<uint32_t> remapLabel(const uint32_t from) const;

  inline bool empty() const { return label_remapping_.empty(); }

  inline operator bool() const { return empty(); }

 private:
  std::map<uint32_t, uint32_t> label_remapping_;
};

}  // namespace hydra
