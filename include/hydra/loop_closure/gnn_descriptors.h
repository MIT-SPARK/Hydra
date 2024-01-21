/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once
#include <config_utilities/virtual_config.h>

#include <filesystem>

#include "hydra/gnn/gnn_interface.h"
#include "hydra/loop_closure/graph_descriptor_factory.h"
#include "hydra/loop_closure/subgraph_extraction.h"

namespace hydra::lcd {

struct LabelEmbeddings {
  using Ptr = std::shared_ptr<LabelEmbeddings>;

  std::map<uint8_t, Eigen::VectorXf> embeddings;
  size_t embedding_size = 0;

  virtual ~LabelEmbeddings() {}

  bool valid() const;

  inline bool empty() const { return embedding_size == 0; }

  inline operator bool() const { return !empty() && valid(); }

  virtual void init() = 0;
};

struct OneHotLabelEmbeddings : LabelEmbeddings {
  struct Config {
    size_t encoding_dim = 0;
  };

  explicit OneHotLabelEmbeddings(const Config& config);

  void init() override;

  const Config config;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<LabelEmbeddings, OneHotLabelEmbeddings, Config>(
          "OneHotLabelEmbeddings");
};

void declare_config(OneHotLabelEmbeddings::Config& config);

struct LabelEmbeddingsFromFile : LabelEmbeddings {
  struct Config {
    std::filesystem::path embedding_path;
  };

  explicit LabelEmbeddingsFromFile(const Config& config);

  void init() override;

  const Config config;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<LabelEmbeddings, LabelEmbeddingsFromFile, Config>(
          "LabelEmbeddingsFromFile");
};

void declare_config(LabelEmbeddingsFromFile::Config& config);

class ObjectGnnDescriptorFactory : GraphDescriptorFactory {
 public:
  struct Config {
    std::filesystem::path model_path;
    SubgraphConfig subgraph;
    double max_edge_distance_m = 5.0;
    config::VirtualConfig<LabelEmbeddings> embeddings;
    bool use_pos_in_feature = false;
  };

  explicit ObjectGnnDescriptorFactory(const Config& config);

  gnn::TensorMap makeInput(const DynamicSceneGraph& graph,
                           const std::set<NodeId>& nodes) const;

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

 public:
  const Config config;

 protected:
  std::unique_ptr<gnn::GnnInterface> model_;
  LabelEmbeddings::Ptr label_embeddings_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphDescriptorFactory,
                                     ObjectGnnDescriptorFactory,
                                     Config>("ObjectGnnDescriptor");
};

void declare_config(ObjectGnnDescriptorFactory::Config& config);

struct PlaceGnnDescriptorFactory : GraphDescriptorFactory {
  struct Config {
    std::filesystem::path model_path;
    SubgraphConfig subgraph;
    bool use_pos_in_feature = false;
  };

  explicit PlaceGnnDescriptorFactory(const Config& config);

  gnn::TensorMap makeInput(const DynamicSceneGraph& graph,
                           const std::set<NodeId>& nodes) const;

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

 public:
  const Config config;

 protected:
  std::unique_ptr<gnn::GnnInterface> model_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<GraphDescriptorFactory,
                                     PlaceGnnDescriptorFactory,
                                     Config>("PlaceGnnDescriptor");
};

void declare_config(PlaceGnnDescriptorFactory::Config& config);

}  // namespace hydra::lcd
