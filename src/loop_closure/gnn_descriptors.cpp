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

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <deque>

namespace hydra::lcd {

using Dsg = DynamicSceneGraph;
using DsgNode = DynamicSceneGraphNode;

ObjectGnnDescriptor::ObjectGnnDescriptor(const std::string& model_path,
                                         const SubgraphConfig& config,
                                         double max_edge_distance_m,
                                         const LabelEmbeddings& label_embeddings,
                                         bool use_pos_in_feature)
    : config_(config),
      max_edge_distance_m_(max_edge_distance_m),
      label_embeddings_(label_embeddings),
      use_pos_in_feature_(use_pos_in_feature) {
  if (label_embeddings_.empty()) {
    throw std::runtime_error("non-empty label embeddings required");
  }

  label_embedding_size_ = label_embeddings_.begin()->second.size();
  for (const auto& label_embedding_pair : label_embeddings_) {
    if (label_embedding_pair.second.size() != static_cast<int>(label_embedding_size_)) {
      std::stringstream ss;
      ss << "label embedding does not agree for label "
         << static_cast<int>(label_embedding_pair.first) << " with size "
         << label_embedding_pair.second.size() << " (detected size "
         << label_embedding_size_ << ")";
      throw std::runtime_error(ss.str());
    }
  }

  model_.reset(new gnn::GnnInterface(model_path));
}

gnn::TensorMap ObjectGnnDescriptor::makeInput(const DynamicSceneGraph& graph,
                                              const std::set<NodeId>& nodes) const {
  const size_t feature_size = label_embedding_size_ + (use_pos_in_feature_ ? 6 : 3);
  gnn::Tensor x(nodes.size(), feature_size, gnn::Tensor::Type::FLOAT32);
  auto x_map = x.map<float>();

  gnn::Tensor pos;
  if (!use_pos_in_feature_) {
    pos = gnn::Tensor(nodes.size(), 3, gnn::Tensor::Type::FLOAT32);
  }
  auto pos_map = pos.map<float>();

  size_t index = 0;
  std::map<NodeId, size_t> index_mapping;
  for (const auto node : nodes) {
    const auto& attrs = graph.getNode(node)->get().attributes<SemanticNodeAttributes>();

    size_t start_idx = 0;
    if (use_pos_in_feature_) {
      start_idx = 3;
      x_map.block(index, 0, 1, 3) = attrs.position.cast<float>().transpose();
    } else {
      pos_map.block(index, 0, 1, 3) = attrs.position.cast<float>().transpose();
    }

    x_map.block(index, start_idx, 1, 3) = attrs.bounding_box.world_P_center.transpose();

    auto iter = label_embeddings_.find(attrs.semantic_label);
    if (iter != label_embeddings_.end()) {
      x_map.block(index, start_idx + 3, 1, label_embedding_size_) =
          iter->second.transpose();
    } else {
      x_map.block(index, start_idx + 3, 1, label_embedding_size_).setZero();
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
      if ((source_position - target_position).norm() > max_edge_distance_m_) {
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

  if (use_pos_in_feature_) {
    return {{"x", x}, {"edge_index", edge_index}};
  } else {
    return {{"x", x}, {"edge_index", edge_index}, {"pos", pos}};
  }
}

Descriptor::Ptr ObjectGnnDescriptor::construct(const Dsg& graph,
                                               const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->nodes = getSubgraphNodes(config_, graph, *parent, false);
  descriptor->root_node = *parent;
  descriptor->root_position = graph.getPosition(*parent);
  descriptor->timestamp = agent_node.timestamp;

  if (descriptor->nodes.empty()) {
    descriptor->is_null = true;
    return descriptor;
  }

  auto input = makeInput(graph, descriptor->nodes);
  VLOG(20) << "Inputs:";
  VLOG(20) << "  - x: " << std::endl << input.at("x").map<float>();
  VLOG(20) << "  - edge_index: " << std::endl << input.at("edge_index").map<int64_t>();
  if (!use_pos_in_feature_) {
    VLOG(20) << "  - pos: " << std::endl << input.at("pos").map<float>();
  }

  auto output = (*model_)(input).at("output");
  VLOG(20) << "--------------------------------------";
  VLOG(20) << "Output type: " << output;
  VLOG(20) << "Output:" << std::endl << output.map<float>();
  descriptor->values = output.map<float>().transpose();
  return descriptor;
}

PlaceGnnDescriptor::PlaceGnnDescriptor(const std::string& model_path,
                                       const SubgraphConfig& config,
                                       bool use_pos_in_feature)
    : config_(config), use_pos_in_feature_(use_pos_in_feature) {
  model_.reset(new gnn::GnnInterface(model_path));
}

gnn::TensorMap PlaceGnnDescriptor::makeInput(const DynamicSceneGraph& graph,
                                             const std::set<NodeId>& nodes) const {
  gnn::Tensor x(nodes.size(), use_pos_in_feature_ ? 5 : 2, gnn::Tensor::Type::FLOAT32);
  auto x_map = x.map<float>();

  gnn::Tensor pos;
  if (!use_pos_in_feature_) {
    pos = gnn::Tensor(nodes.size(), 3, gnn::Tensor::Type::FLOAT32);
  }
  auto pos_map = pos.map<float>();

  size_t index = 0;
  std::map<NodeId, size_t> index_mapping;
  for (const auto node : nodes) {
    const auto& attrs = graph.getNode(node)->get().attributes<PlaceNodeAttributes>();
    if (use_pos_in_feature_) {
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

  if (use_pos_in_feature_) {
    return {{"x", x}, {"edge_index", edge_index}};
  } else {
    return {{"x", x}, {"edge_index", edge_index}, {"pos", pos}};
  }
}

Descriptor::Ptr PlaceGnnDescriptor::construct(const Dsg& graph,
                                              const DsgNode& agent_node) const {
  auto parent = agent_node.getParent();
  if (!parent) {
    return nullptr;
  }

  auto descriptor = std::make_unique<Descriptor>();
  descriptor->normalized = false;
  descriptor->nodes = getSubgraphNodes(config_, graph, *parent, true);
  descriptor->root_node = *parent;
  descriptor->root_position = graph.getPosition(*parent);
  descriptor->timestamp = agent_node.timestamp;

  if (descriptor->nodes.empty()) {
    return nullptr;
  }

  auto input = makeInput(graph, descriptor->nodes);
  VLOG(20) << "Inputs:";
  VLOG(20) << "  - x: " << std::endl << input.at("x").map<float>();
  VLOG(20) << "  - edge_index: " << std::endl << input.at("edge_index").map<int64_t>();
  if (!use_pos_in_feature_) {
    VLOG(20) << "  - pos: " << std::endl << input.at("pos").map<float>();
  }

  auto output = (*model_)(input).at("output");
  VLOG(20) << "--------------------------------------";
  VLOG(20) << "Output type: " << output;
  VLOG(20) << "Output:" << std::endl << output.map<float>();
  descriptor->values = output.map<float>().transpose();
  return descriptor;
}

ObjectGnnDescriptor::LabelEmbeddings loadLabelEmbeddings(const std::string& filename) {
  ObjectGnnDescriptor::LabelEmbeddings embeddings;
  VLOG(5) << "Reading embeddings from: " << filename;

  YAML::Node root = YAML::LoadFile(filename);
  if (!root["embeddings"]) {
    LOG(ERROR) << "file " << filename << " did not contain embeddings field";
    return embeddings;
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

  return embeddings;
}

}  // namespace hydra::lcd
