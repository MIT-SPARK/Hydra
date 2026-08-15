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

#include "hydra_visualizer/adapters/text.h"

#include <config_utilities/factory.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>
#include <spark_dsg/printing.h>

#include <sstream>

namespace hydra::visualizer {

using namespace spark_dsg;

namespace {

static const auto id_reg =
    config::RegistrationWithConfig<NodeTextAdapter,
                                   IdTextAdapter,
                                   IdTextAdapter::Config>("IdTextAdapter");

static const auto label_reg =
    config::RegistrationWithConfig<NodeTextAdapter,
                                   LabelTextAdapter,
                                   LabelTextAdapter::Config>("LabelTextAdapter");

static const auto label_id_reg =
    config::RegistrationWithConfig<NodeTextAdapter,
                                   LabelIdTextAdapter,
                                   LabelIdTextAdapter::Config>("LabelIdTextAdapter");

static const auto name_reg =
    config::RegistrationWithConfig<NodeTextAdapter,
                                   NameTextAdapter,
                                   NameTextAdapter::Config>("NameTextAdapter");

static const auto name_id_reg =
    config::RegistrationWithConfig<NodeTextAdapter,
                                   NameIdTextAdapter,
                                   NameIdTextAdapter::Config>("NameIdTextAdapter");

static const auto attributes_reg =
    config::RegistrationWithConfig<NodeTextAdapter,
                                   AttributesTextAdaptor,
                                   AttributesTextAdaptor::Config>(
        "AttributesTextAdaptor");

inline std::string getNodeName(const SceneGraphNode& node) {
  const auto attrs = node.tryAttributes<SemanticNodeAttributes>();
  return attrs ? attrs->name : "";
}

inline std::string getNodeLabel(const DynamicSceneGraph& graph,
                                const SceneGraphNode& node,
                                std::map<std::string, Labelspace>& labelspaces) {
  const auto attrs = node.tryAttributes<SemanticNodeAttributes>();
  if (!attrs) {
    return "";
  }

  std::stringstream ss;
  ss << node.layer;
  const auto key = ss.str();

  auto iter = labelspaces.find(key);
  if (iter == labelspaces.end()) {
    iter = labelspaces
               .emplace(key,
                        Labelspace::fromMetadata(
                            graph, node.layer.layer, node.layer.partition))
               .first;
  }

  return iter->second.getCategory(*attrs);
}

}  // namespace

void declare_config(IdTextAdapter::Config&) {}

std::string IdTextAdapter::getText(const DynamicSceneGraph&,
                                   const SceneGraphNode& node) const {
  return NodeSymbol(node.id).str();
}

void declare_config(LabelTextAdapter::Config&) {}

std::string LabelTextAdapter::getText(const DynamicSceneGraph& graph,
                                      const SceneGraphNode& node) const {
  return getNodeLabel(graph, node, labelspaces_);
}

void declare_config(LabelIdTextAdapter::Config&) {}

std::string LabelIdTextAdapter::getText(const DynamicSceneGraph& graph,
                                        const SceneGraphNode& node) const {
  const auto id = NodeSymbol(node.id).str();
  const auto label = getNodeLabel(graph, node, labelspaces_);
  return label.empty() ? id : label + " : " + id;
}

void declare_config(NameTextAdapter::Config&) {}

std::string NameTextAdapter::getText(const DynamicSceneGraph&,
                                     const SceneGraphNode& node) const {
  return getNodeName(node);
}

void declare_config(NameIdTextAdapter::Config&) {}

std::string NameIdTextAdapter::getText(const DynamicSceneGraph&,
                                       const SceneGraphNode& node) const {
  const auto id = NodeSymbol(node.id).str();
  const auto name = getNodeName(node);
  return name.empty() ? id : name + " : " + id;
}

void declare_config(AttributesTextAdaptor::Config&) {}

std::string AttributesTextAdaptor::getText(const DynamicSceneGraph&,
                                           const SceneGraphNode& node) const {
  std::stringstream ss;
  ss << node.attributes();
  return ss.str();
}

}  // namespace hydra::visualizer
