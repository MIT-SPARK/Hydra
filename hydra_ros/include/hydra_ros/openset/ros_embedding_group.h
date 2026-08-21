#pragma once
#include <hydra/openset/embedding_group.h>
#include <hydra/utils/logging.h>

namespace hydra {

struct RosEmbeddingGroup : public EmbeddingGroup {
  struct Config : VerbosityConfig {
    Config();

    std::string ns = "~";
    size_t prompt_timeout_ms = 0;
    std::vector<std::string> prompts;
  };

  explicit RosEmbeddingGroup(const Config& config);
};

void declare_config(RosEmbeddingGroup::Config& config);

}  // namespace hydra
