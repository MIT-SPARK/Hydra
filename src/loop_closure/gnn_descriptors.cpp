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
#include "hydra/loop_closure/gnn_descriptors.h"

#include <config_utilities/config.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <deque>

#include "hydra/common/common.h"

namespace hydra::lcd {

using Dsg = DynamicSceneGraph;
using DsgNode = DynamicSceneGraphNode;
using ObjectGnnFactory = ObjectGnnDescriptorFactory;
using PlaceGnnFactory = PlaceGnnDescriptorFactory;

bool LabelEmbeddings::valid() const {
  for (auto&& [label, embedding] : embeddings) {
    if (embedding.size() != static_cast<int>(embedding_size)) {
      LOG(ERROR) << "label embedding does not agree for label "
                 << static_cast<int>(label) << " with size " << embedding.size()
                 << " (detected size " << embedding_size << ")";
      return false;
    }
  }
  return true;
}

OneHotLabelEmbeddings::OneHotLabelEmbeddings(const Config& config)
    : config(config::checkValid(config)) {}

void OneHotLabelEmbeddings::init() {
  for (size_t i = 0; i < config.encoding_dim; ++i) {
    Eigen::VectorXf vec = Eigen::VectorXf::Zero(config.encoding_dim);
    vec(i) = 1.0f;
    embeddings[static_cast<uint8_t>(i)] = vec;
  }
}

void declare_config(OneHotLabelEmbeddings::Config& config) {
  using namespace config;
  name("OneHotLabelEmbeddings::Config");
  field(config.encoding_dim, "encoding_dim");
  check(config.encoding_dim, GT, 0, "encoding_dim");
}

LabelEmbeddingsFromFile::LabelEmbeddingsFromFile(const Config& config)
    : config(config::checkValid(config)) {}

void LabelEmbeddingsFromFile::init() {
  const auto filename = config.embedding_path.string();
  VLOG(VLEVEL_FILE) << "Reading embeddings from: '" << filename << "'";

  YAML::Node root = YAML::LoadFile(filename);
  if (!root["embeddings"]) {
    LOG(ERROR) << "file " << filename << " did not contain embeddings field";
    return;
  }

  for (const auto embedding : root["embeddings"]) {
    if (!embedding["label"]) {
      LOG(WARNING) << "embedding did not contain label! skipping...";
      continue;
    }

    if (!embedding["values"]) {
      LOG(WARNING) << "embedding did not contain values! skipping...";
      continue;
    }

    const auto label = embedding["label"].as<int>();
    const auto vec = embedding["values"].as<std::vector<float>>();
    auto iter = embeddings.emplace(label, Eigen::VectorXf(vec.size())).first;
    for (size_t i = 0; i < vec.size(); ++i) {
      iter->second(i) = vec[i];
    }
  }
}

void declare_config(LabelEmbeddingsFromFile::Config& config) {
  using namespace config;
  name("LabelEmbeddingsFromFile::Config");
  field<Path>(config.embedding_path, "embedding_path");
  check<Path::Exists>(config.embedding_path, "embedding_path");
}

ObjectGnnFactory::ObjectGnnDescriptorFactory(const Config& config)
    : config(config::checkValid(config)) {
  label_embeddings_ = config.embeddings.create();
  CHECK(label_embeddings_) << "Label embeddings required!";
  label_embeddings_->init();
  CHECK(!label_embeddings_->empty()) << "Label embeddings empty";
  CHECK(label_embeddings_->valid()) << "Label embeddings invalid";
  model_.reset(new gnn::GnnInterface(config.model_path.string()));
}

gnn::TensorMap ObjectGnnFactory::makeInput(const DynamicSceneGraph& graph,
                                           const std::set<NodeId>& nodes) const {
  const auto embedding_size = label_embeddings_->embedding_size;
  const size_t feature_size = embedding_size + (config.use_pos_in_feature ? 6 : 3);
  gnn::Tensor x(nodes.size(), feature_size, gnn::Tensor::Type::FLOAT32);
  auto x_map = x.map<float>();

  gnn::Tensor pos;
  if (!config.use_pos_in_feature) {
    pos = gnn::Tensor(nodes.size(), 3, gnn::Tensor::Type::FLOAT32);
  }
  auto pos_map = pos.map<float>();

  size_t index = 0;
  std::map<NodeId, size_t> index_mapping;
  for (const auto node : nodes) {
    const auto& attrs = graph.getNode(node)->get().attributes<SemanticNodeAttributes>();

    size_t start_idx = 0;
    if (config.use_pos_in_feature) {
      start_idx = 3;
      x_map.block(index, 0, 1, 3) = attrs.position.cast<float>().transpose();
    } else {
      pos_map.block(index, 0, 1, 3) = attrs.position.cast<float>().transpose();
    }

    x_map.block(index, start_idx, 1, 3) =
        (attrs.bounding_box.max - attrs.bounding_box.min).transpose();

    auto iter = label_embeddings_->embeddings.find(attrs.semantic_label);
    if (iter != label_embeddings_->embeddings.end()) {
      x_map.block(index, start_idx + 3, 1, embedding_size) = iter->second.transpose();
    } else {
      x_map.block(index, start_idx + 3, 1, embedding_size).setZero();
    }

    index_mapping[node] = index;
    ++index;
  }

  std::list<std::pair<int64_t, int64_t>> edges;
  for (const auto& source : nodes) {
    const auto source_position = graph.getPosition(source);
    for (const auto& target : nodes) {
      if (source == target) {
        continue;
      }

      const auto target_position = graph.getPosition(target);
      if ((source_position - target_position).norm() > config.max_edge_distance_m) {
        continue;
      }

      edges.push_back({index_mapping[source], index_mapping[target]});
    }
  }

  // note that edges is built in an undirected manner, so edges.size() = 2 * |E|
  gnn::Tensor edge_index(2, edges.size(), gnn::Tensor::Type::INT64);
  auto edge_map = edge_index.map<int64_t>();
  size_t edge_idx = 0;
  for (const auto& edge : edges) {
    edge_map(0, edge_idx) = edge.first;
    edge_map(1, edge_idx) = edge.second;
    ++edge_idx;
  }

  if (config.use_pos_in_feature) {
    return {{"x", x}, {"edge_index", edge_index}};
  } else {
    return {{"x", x}, {"edge_index", edge_index}, {"pos", pos}};
  }
}

Descriptor::Ptr ObjectGnnFactory::construct(const Dsg& graph,
                                            const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->nodes = getSubgraphNodes(config.subgraph, graph, *parent, false);
  descriptor->root_node = *parent;
  descriptor->root_position = graph.getPosition(*parent);
  descriptor->timestamp = agent_node.timestamp;

  if (descriptor->nodes.empty()) {
    descriptor->is_null = true;
    return descriptor;
  }

  auto input = makeInput(graph, descriptor->nodes);
  VLOG(VLEVEL_ALL) << "Inputs:";
  VLOG(VLEVEL_ALL) << "  - x: " << std::endl << input.at("x").map<float>();
  VLOG(VLEVEL_ALL) << "  - edge_index: " << std::endl
                   << input.at("edge_index").map<int64_t>();
  if (!config.use_pos_in_feature) {
    VLOG(VLEVEL_ALL) << "  - pos: " << std::endl << input.at("pos").map<float>();
  }

  auto output = (*model_)(input).at("output");
  VLOG(VLEVEL_ALL) << "--------------------------------------";
  VLOG(VLEVEL_ALL) << "Output type: " << output;
  VLOG(VLEVEL_ALL) << "Output:" << std::endl << output.map<float>();
  descriptor->values = output.map<float>().transpose();
  return descriptor;
}

void declare_config(ObjectGnnDescriptorFactory::Config& config) {
  using namespace config;
  name("ObjectGnnDescriptorFactory::Config");
  field<Path>(config.model_path, "model_path");
  field(config.subgraph, "subgraph");
  field(config.max_edge_distance_m, "max_edge_distance_m");
  field(config.embeddings, "embeddings");
  field(config.use_pos_in_feature, "use_pos_in_feature");
  check<Path::Exists>(config.model_path, "model_path");
  check(config.max_edge_distance_m, GT, 0.0, "max_edge_distance_m positive");
}

PlaceGnnFactory::PlaceGnnDescriptorFactory(const Config& config)
    : config(config::checkValid(config)) {
  model_.reset(new gnn::GnnInterface(config.model_path.string()));
}

gnn::TensorMap PlaceGnnFactory::makeInput(const DynamicSceneGraph& graph,
                                          const std::set<NodeId>& nodes) const {
  gnn::Tensor x(
      nodes.size(), config.use_pos_in_feature ? 5 : 2, gnn::Tensor::Type::FLOAT32);
  auto x_map = x.map<float>();

  gnn::Tensor pos;
  if (!config.use_pos_in_feature) {
    pos = gnn::Tensor(nodes.size(), 3, gnn::Tensor::Type::FLOAT32);
  }
  auto pos_map = pos.map<float>();

  size_t index = 0;
  std::map<NodeId, size_t> index_mapping;
  for (const auto node : nodes) {
    const auto& attrs = graph.getNode(node)->get().attributes<PlaceNodeAttributes>();
    if (config.use_pos_in_feature) {
      x_map.block(index, 0, 1, 3) = attrs.position.cast<float>().transpose();
      x_map(index, 3) = attrs.distance;
      x_map(index, 4) = static_cast<float>(attrs.num_basis_points);
    } else {
      pos_map.block(index, 0, 1, 3) = attrs.position.cast<float>().transpose();
      x_map(index, 0) = attrs.distance;
      x_map(index, 1) = static_cast<float>(attrs.num_basis_points);
    }
    index_mapping[node] = index;
    ++index;
  }

  std::list<std::pair<int64_t, int64_t>> edges;
  for (const auto source : nodes) {
    const SceneGraphNode& node = *graph.getNode(source);
    for (const auto sibling : node.siblings()) {
      if (!nodes.count(sibling)) {
        continue;
      }

      edges.push_back({index_mapping[source], index_mapping[sibling]});
    }
  }

  // note that edges is built in an undirected manner, so edges.size() = 2 * |E|
  gnn::Tensor edge_index(2, edges.size(), gnn::Tensor::Type::INT64);
  auto edge_map = edge_index.map<int64_t>();
  size_t edge_idx = 0;
  for (const auto& edge : edges) {
    edge_map(0, edge_idx) = edge.first;
    edge_map(1, edge_idx) = edge.second;
    ++edge_idx;
  }

  if (config.use_pos_in_feature) {
    return {{"x", x}, {"edge_index", edge_index}};
  } else {
    return {{"x", x}, {"edge_index", edge_index}, {"pos", pos}};
  }
}

Descriptor::Ptr PlaceGnnFactory::construct(const Dsg& graph,
                                           const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->nodes = getSubgraphNodes(config.subgraph, graph, *parent, true);
  descriptor->root_node = *parent;
  descriptor->root_position = graph.getPosition(*parent);
  descriptor->timestamp = agent_node.timestamp;

  if (descriptor->nodes.empty()) {
    return nullptr;
  }

  auto input = makeInput(graph, descriptor->nodes);
  VLOG(VLEVEL_ALL) << "Inputs:";
  VLOG(VLEVEL_ALL) << "  - x: " << std::endl << input.at("x").map<float>();
  VLOG(VLEVEL_ALL) << "  - edge_index: " << std::endl
                   << input.at("edge_index").map<int64_t>();
  if (!config.use_pos_in_feature) {
    VLOG(VLEVEL_ALL) << "  - pos: " << std::endl << input.at("pos").map<float>();
  }

  auto output = (*model_)(input).at("output");
  VLOG(VLEVEL_ALL) << "--------------------------------------";
  VLOG(VLEVEL_ALL) << "Output type: " << output;
  VLOG(VLEVEL_ALL) << "Output:" << std::endl << output.map<float>();
  descriptor->values = output.map<float>().transpose();
  return descriptor;
}

void declare_config(PlaceGnnDescriptorFactory::Config& config) {
  using namespace config;
  name("PlaceGnnDescriptorFactory::Config");
  field<Path>(config.model_path, "model_path");
  field(config.subgraph, "subgraph");
  field(config.use_pos_in_feature, "use_pos_in_feature");
  check<Path::Exists>(config.model_path, "model_path");
}

}  // namespace hydra::lcd
