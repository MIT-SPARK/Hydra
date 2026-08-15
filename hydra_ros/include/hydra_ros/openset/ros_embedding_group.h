#pragma once
#include <config_utilities/factory.h>
#include <hydra/openset/embedding_group.h>

namespace hydra {

struct RosEmbeddingGroup : public EmbeddingGroup {
  struct Config {
    std::string ns = "~";
    bool silent_wait = false;
    std::vector<std::string> prompts;
  };

  explicit RosEmbeddingGroup(const Config& config);

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<EmbeddingGroup, RosEmbeddingGroup, Config>(
          "RosEmbeddingGroup");
};

void declare_config(RosEmbeddingGroup::Config& config);

}  // namespace hydra
