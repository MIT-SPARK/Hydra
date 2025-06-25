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
#include <memory>

#include "hydra/common/dsg_types.h"
#include "hydra/common/output_sink.h"
#include "hydra/frontend/mesh_segmenter.h"
#include "hydra/reconstruction/volumetric_map.h"

namespace hydra {

class ObjectExtractor {
 public:
  using Sink = OutputSink<uint64_t, const ObjectExtractor&>;

  struct Config {
    //! Layer to add objects to
    std::string layer_id = DsgLayers::OBJECTS;
    //! Bounding box type to fit
    BoundingBox::Type bounding_box_type = BoundingBox::Type::AABB;
    //! Segmentation config
    MeshSegmenter::Config mesh_segmenter;
    //! Visualization sinks
    std::vector<Sink::Factory> sinks;
  } const config;

  explicit ObjectExtractor(const Config& config, const std::set<uint32_t>& labels);

  void detect(uint64_t timestamp_ns, const VolumetricMap& map);

  void updateGraph(uint64_t timestamp, DynamicSceneGraph& graph);

 private:
  void updateOldNodes(const VolumetricMap& map);

  void mergeActiveNodes();

 private:
  const Sink::List sinks_;
  const MeshSegmenter segmenter_;

  NodeSymbol next_node_id_;
  std::set<NodeId> removed_nodes_;
  std::map<uint32_t, std::map<NodeId, ObjectNodeAttributes::Ptr>> active_nodes_;
};

void declare_config(ObjectExtractor::Config& config);

}  // namespace hydra
