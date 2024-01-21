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

#include "hydra/loop_closure/descriptor_matching.h"
#include "hydra/loop_closure/graph_descriptor_factory.h"
#include "hydra/loop_closure/graph_registration.h"
#include "hydra/loop_closure/sensor_descriptor_factory.h"
#include "hydra/loop_closure/sensor_registration.h"

namespace hydra::lcd {

struct LayerLcdConfig {
  LayerId layer;
  config::VirtualConfig<GraphDescriptorFactory> descriptors;
  DescriptorMatchConfig matching;
  config::VirtualConfig<GraphRegistrationSolver> registration;
};

void declare_config(LayerLcdConfig& conf);

struct SensorLcdConfig {
  config::VirtualConfig<SensorDescriptorFactory> descriptors;
  DescriptorMatchConfig matching;
  config::VirtualConfig<SensorRegistrationSolver> registration;
};

void declare_config(SensorLcdConfig& conf);

struct LcdDetectorConfig {
  SensorLcdConfig agent_config;
  std::vector<LayerLcdConfig> graph_configs;
};

void declare_config(LcdDetectorConfig& conf);

class LcdDetector {
 public:
  using FactoryMap = std::map<LayerId, GraphDescriptorFactory::Ptr>;
  using SearchResultMap = std::map<size_t, LayerSearchResults>;

  explicit LcdDetector(const LcdDetectorConfig& config);

  void addSensorDescriptor(const Sensor& sensor,
                           const DynamicSceneGraph& graph,
                           const NodeId agent_id,
                           const FrameData& data);

  void updateDescriptorCache(const DynamicSceneGraph& graph,
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

 public:
  const LcdDetectorConfig config;

 protected:
  Descriptor* getAgentDescriptor(const DynamicSceneGraphNode& node);

  RegistrationSolution registerAgent(const DynamicSceneGraph& graph,
                                     const LayerSearchResults& match,
                                     NodeId agent_id,
                                     uint64_t timestamp) const;

  std::vector<RegistrationSolution> registerAndVerify(const DynamicSceneGraph& dsg,
                                                      const SearchResultMap& matches,
                                                      NodeId agent_node,
                                                      uint64_t timestamp = 0) const;

  SensorDescriptorFactory::Ptr agent_factory_;
  FactoryMap layer_factories_;

  LayerId root_layer_;
  size_t max_internal_index_;
  std::map<size_t, LayerId> internal_index_to_layer_;
  std::map<size_t, DescriptorMatchConfig> match_config_map_;

  SensorRegistrationSolver::Ptr agent_registration_;
  std::map<size_t, GraphRegistrationSolver::Ptr> registration_solvers_;

  std::map<LayerId, DescriptorCache> cache_map_;
  std::map<NodeId, DescriptorCache> leaf_cache_;
  std::map<NodeId, std::set<NodeId>> root_leaf_map_;
  std::map<NodeId, Descriptor::Ptr> queued_agent_descriptors_;
  std::map<NodeId, SensorFeatures::Ptr> agent_features_;

  std::map<size_t, LayerSearchResults> matches_;
};

}  // namespace hydra::lcd
