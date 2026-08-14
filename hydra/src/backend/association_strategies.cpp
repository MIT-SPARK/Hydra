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

#include "hydra/backend/association_strategies.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <glog/logging.h>

namespace hydra::association {
namespace {

// NOTE(nathan) const references are required because otherwise factory uses copy
// constructor

static const auto pairwise_reg =
    config::RegistrationWithConfig<AssociationStrategy,
                                   Pairwise,
                                   Pairwise::Config,
                                   const SceneGraphLayer&>("Pairwise");

static const auto semantic_pairwise_reg =
    config::RegistrationWithConfig<AssociationStrategy,
                                   SemanticPairwise,
                                   SemanticPairwise::Config,
                                   const SceneGraphLayer&>("SemanticPairwise");

static const auto nearest_node_reg =
    config::RegistrationWithConfig<AssociationStrategy,
                                   NearestNode,
                                   NearestNode::Config,
                                   const SceneGraphLayer&>("NearestNode");

static const auto semantic_nearest_node_reg =
    config::RegistrationWithConfig<AssociationStrategy,
                                   SemanticNearestNode,
                                   SemanticNearestNode::Config,
                                   const SceneGraphLayer&>("SemanticNearestNode");

}  // namespace

void declare_config(Pairwise::Config&) { config::name("Pairwise::Config"); }

LayerView Pairwise::candidates(const SceneGraphLayer& layer,
                               const SceneGraphNode& node) const {
  return LayerView(layer, [&node](const auto& other) {
    if (other.id == node.id) {
      return false;
    }

    // NOTE(nathan) this results in some of the nodes on the boundary of the active
    // window being considered as merge targets and sources, but it should be fine
    return !other.attributes().is_active;
  });
}

void declare_config(SemanticPairwise::Config&) {
  config::name("SemanticPairwise::Config");
}

LayerView SemanticPairwise::candidates(const SceneGraphLayer& layer,
                                       const SceneGraphNode& node) const {
  return LayerView(layer, [&node](const SceneGraphNode& other) {
    if (node.id == other.id) {
      return false;
    }

    const auto& lhs_attrs = node.attributes<SemanticNodeAttributes>();
    const auto& rhs_attrs = other.attributes<SemanticNodeAttributes>();
    if (rhs_attrs.is_active) {
      // NOTE(nathan) we don't catch all "active window" nodes with this, but it should
      // be fine
      return false;
    }

    return lhs_attrs.semantic_label == rhs_attrs.semantic_label;
  });
}

void declare_config(NearestNode::Config& config) {
  using namespace config;
  name("NearestNode::Config");
  field(config.num_merges_to_consider, "num_merges_to_consider");
}

NearestNode::NearestNode(const Config& config, const SceneGraphLayer& layer)
    : config(config) {
  // TODO(nathan) this used to filter by real_place, but there's no way to do that.
  // (it should be fine generally, but might cause weirdness with the frontiers)
  node_finder = NearestNodeFinder::fromLayer(
      layer, [](const SceneGraphNode& node) { return !node.attributes().is_active; });
}

NearestNode::~NearestNode() = default;

LayerView NearestNode::candidates(const SceneGraphLayer& layer,
                                  const SceneGraphNode& node) const {
  if (!node_finder) {
    return {};
  }

  const auto& from_attrs = node.attributes();

  std::set<NodeId> candidates;
  node_finder->find(
      from_attrs.position,
      config.num_merges_to_consider,
      !from_attrs.is_active,
      [&candidates](NodeId place_id, size_t, double) { candidates.insert(place_id); });

  // NOTE(nathan) bind by copy required (otherwise reference goes out of scope...)
  return LayerView(
      layer, [candidates](const auto& node) { return candidates.count(node.id); });
}

void declare_config(SemanticNearestNode::Config& config) {
  using namespace config;
  name("SemanticNearestNode::Config");
  field(config.num_merges_to_consider, "num_merges_to_consider");
}

SemanticNearestNode::SemanticNearestNode(const Config& config,
                                         const SceneGraphLayer& layer)
    : config(config) {
  makeSemanticNodeFinders(layer, node_finders);
}

SemanticNearestNode::~SemanticNearestNode() = default;

LayerView SemanticNearestNode::candidates(const SceneGraphLayer& layer,
                                          const SceneGraphNode& node) const {
  const auto& from_attrs = node.attributes<SemanticNodeAttributes>();
  const auto iter = node_finders.find(from_attrs.semantic_label);
  if (iter == node_finders.end()) {
    return {};
  }

  std::set<NodeId> candidates;
  (*iter).second->find(from_attrs.position,
                       config.num_merges_to_consider,
                       !from_attrs.is_active,
                       [&candidates](NodeId object_id, size_t, double) {
                         candidates.insert(object_id);
                       });

  // NOTE(nathan) bind by copy required (otherwise reference goes out of scope...)
  return LayerView(
      layer, [candidates](const auto& node) { return candidates.count(node.id); });
}

}  // namespace hydra::association
