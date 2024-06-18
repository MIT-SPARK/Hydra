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
#include "hydra/loop_closure/descriptor_matching.h"
#include "hydra/loop_closure/registration.h"
#include "hydra/loop_closure/scene_graph_descriptors.h"

namespace hydra::lcd {

struct GnnLcdConfig {
  bool use_onehot_encoding = true;
  size_t onehot_encoding_dim = 39;
  std::string label_embeddings_file;
  double object_connection_radius_m;
  std::string object_model_path;
  std::string places_model_path;
  bool places_pos_in_feature = false;
  bool objects_pos_in_feature = false;
};

struct LayerLcdConfig {
  DescriptorMatchConfig matching;
  LayerRegistrationConfig registration;
};

struct LcdDetectorConfig {
  TeaserParams teaser_config;
  bool enable_agent_registration = true;
  DescriptorMatchConfig agent_search_config;

  // TODO(nathan) refactor this
  LayerLcdConfig objects;
  LayerLcdConfig places;

  SubgraphConfig object_extraction{5.0};
  SubgraphConfig places_extraction{5.0};

  size_t num_semantic_classes = 20;
  HistogramConfig<double> place_histogram_config{0.5, 2.5, 30};
  bool use_gnn_descriptors = false;
  GnnLcdConfig gnn_lcd;
};

class LcdDetector {
 public:
  using FactoryMap = std::map<LayerId, DescriptorFactory::Ptr>;
  using SearchResultMap = std::map<size_t, LayerSearchResults>;

  explicit LcdDetector(const LcdDetectorConfig& config);

  void setDescriptorFactories(FactoryMap&& factories);

  void setRegistrationSolver(size_t level, DsgRegistrationSolver::Ptr&& solver);

  void updateDescriptorCache(const DynamicSceneGraph& dsg,
                             const std::unordered_set<NodeId>& archived_places,
                             uint64_t timestamp = 0);

  std::vector<RegistrationSolution> detect(const DynamicSceneGraph& dsg,
                                           NodeId latest_agent_id,
                                           uint64_t timestamp = 0);

  size_t numDescriptors() const;

  size_t numGraphDescriptors(LayerId layer) const;

  size_t numAgentDescriptors() const;

  const SearchResultMap& getLatestMatches() const;

  const std::map<LayerId, size_t>& getLayerRemapping() const;

  const DescriptorCache& getDescriptorCache(LayerId layer);

  void dumpDescriptors(const std::string& log_path) const;

 protected:
  void makeDefaultDescriptorFactories();

  void resetLayerAssignments(
      const std::map<LayerId, DescriptorMatchConfig>& match_configs,
      const std::map<LayerId, LayerRegistrationConfig>& reg_configs);

  bool addNewDescriptors(const DynamicSceneGraph& graph,
                         const SceneGraphNode& agent_node);

  std::vector<RegistrationSolution> registerAndVerify(const DynamicSceneGraph& dsg,
                                                      const SearchResultMap& matches,
                                                      NodeId agent_node,
                                                      uint64_t timestamp = 0) const;

  LcdDetectorConfig config_;
  DescriptorFactory::Ptr agent_factory_;
  FactoryMap layer_factories_;

  LayerId root_layer_;
  size_t max_internal_index_;
  std::map<LayerId, size_t> layer_to_internal_index_;
  std::map<size_t, LayerId> internal_index_to_layer_;

  std::map<size_t, DescriptorMatchConfig> match_config_map_;
  std::map<size_t, DsgRegistrationSolver::Ptr> registration_solvers_;
  // std::map<size_t, ValidationFunc> validation_funcs_;

  std::map<LayerId, DescriptorCache> cache_map_;
  std::map<NodeId, DescriptorCache> leaf_cache_;
  std::map<NodeId, std::set<NodeId>> root_leaf_map_;

  std::map<size_t, LayerSearchResults> matches_;
};

}  // namespace hydra::lcd
