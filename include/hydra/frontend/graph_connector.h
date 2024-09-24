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
#include <spark_dsg/layer_prefix.h>
#include <spark_dsg/scene_graph_types.h>

#include <map>
#include <memory>
#include <set>
#include <vector>

namespace spark_dsg {
class DynamicSceneGraph;
}

namespace hydra {

struct LayerConnector {
  struct Config {
    struct ChildLayerConfig {
      spark_dsg::LayerId layer = spark_dsg::DsgLayers::UNKNOWN;
      bool include_static = true;
      bool include_dynamic = true;
    };
    // TODO(nathan) for implementation reasons, the parent has to be static
    spark_dsg::LayerId parent_layer = spark_dsg::DsgLayers::PLACES;
    std::vector<ChildLayerConfig> child_layers{
        {spark_dsg::DsgLayers::OBJECTS, true, true}};
    size_t verbosity = 0;
  } const config;

  explicit LayerConnector(const Config& config);

  bool isChild(const spark_dsg::LayerKey& key) const;

  void updateParents(const spark_dsg::DynamicSceneGraph& graph,
                     const std::vector<spark_dsg::NodeId>& new_nodes);

  void connectChildren(spark_dsg::DynamicSceneGraph& graph,
                       const std::vector<spark_dsg::NodeId>& new_nodes);

  // config
  const spark_dsg::LayerKey parent_layer;
  std::set<spark_dsg::LayerId> static_child_layers;
  std::set<spark_dsg::LayerId> dynamic_child_layers;
  // tracking
  std::set<spark_dsg::NodeId> active_children;
  std::map<spark_dsg::NodeId, std::set<spark_dsg::NodeId>> active_parents;
};

void declare_config(LayerConnector::Config::ChildLayerConfig& config);

void declare_config(LayerConnector::Config& config);

class GraphConnector {
 public:
  struct Config {
    std::vector<LayerConnector::Config> layers{LayerConnector::Config{}};
  } const config;

  explicit GraphConnector(const Config& config);

  virtual ~GraphConnector();

  void connect(spark_dsg::DynamicSceneGraph& graph);

 protected:
  std::vector<LayerConnector> layers_;
};

void declare_config(GraphConnector::Config& config);

}  // namespace hydra
