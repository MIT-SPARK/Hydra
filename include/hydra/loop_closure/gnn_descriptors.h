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
#include "hydra/gnn/gnn_interface.h"
#include "hydra/loop_closure/scene_graph_descriptors.h"

namespace hydra::lcd {

struct ObjectGnnDescriptor : DescriptorFactory {
  using LabelEmbeddings = std::map<uint8_t, Eigen::VectorXf>;

  ObjectGnnDescriptor(const std::string& model_path,
                      const SubgraphConfig& config,
                      double max_edge_distance_m,
                      const LabelEmbeddings& label_embeddings,
                      bool use_pos_in_feature = true);

  gnn::TensorMap makeInput(const DynamicSceneGraph& graph,
                           const std::set<NodeId>& nodes) const;

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

 protected:
  const SubgraphConfig config_;
  std::unique_ptr<gnn::GnnInterface> model_;

  double max_edge_distance_m_;
  size_t label_embedding_size_;
  std::map<uint8_t, Eigen::VectorXf> label_embeddings_;
  const bool use_pos_in_feature_;
};

struct PlaceGnnDescriptor : DescriptorFactory {
  PlaceGnnDescriptor(const std::string& model_path,
                     const SubgraphConfig& config,
                     bool use_pos_in_feature = true);

  gnn::TensorMap makeInput(const DynamicSceneGraph& graph,
                           const std::set<NodeId>& nodes) const;

  Descriptor::Ptr construct(const DynamicSceneGraph& graph,
                            const DynamicSceneGraphNode& agent_node) const override;

 protected:
  const SubgraphConfig config_;
  const bool use_pos_in_feature_;
  std::unique_ptr<gnn::GnnInterface> model_;
};

ObjectGnnDescriptor::LabelEmbeddings loadLabelEmbeddings(const std::string& filename);

}  // namespace hydra::lcd
