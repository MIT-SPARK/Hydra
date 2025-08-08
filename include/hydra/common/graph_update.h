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
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include <list>
#include <map>

#include "hydra/common/node_matchers.h"

namespace spark_dsg {
class DynamicSceneGraph;
}

namespace hydra {

struct LayerUpdate {
  using Ptr = std::shared_ptr<LayerUpdate>;
  explicit LayerUpdate(spark_dsg::LayerId layer);
  void append(LayerUpdate&& other);

  const spark_dsg::LayerId layer;
  std::list<spark_dsg::NodeAttributes::Ptr> attributes;
};

using GraphUpdate = std::map<spark_dsg::LayerId, LayerUpdate::Ptr>;

struct LayerTracker {
  struct Config {
    char prefix = 0;
    std::optional<spark_dsg::LayerId> target_layer;
    config::VirtualConfig<NodeMatcher> matcher;
  } const config;

  explicit LayerTracker(const Config& config);

  spark_dsg::NodeSymbol next_id;
  std::unique_ptr<NodeMatcher> matcher;
};

void declare_config(LayerTracker::Config& config);

struct GraphUpdater {
  struct Config {
    std::map<std::string, LayerTracker::Config> layer_updates;
    //! @brief mark all added nodes as active
    bool mark_active = true;
  } const config;

  explicit GraphUpdater(const Config& config);
  void update(const GraphUpdate& update, spark_dsg::DynamicSceneGraph& graph);

 private:
  std::map<std::string, LayerTracker> trackers_;
};

void declare_config(GraphUpdater::Config& config);

}  // namespace hydra
