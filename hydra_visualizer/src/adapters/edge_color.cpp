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
#include "hydra_visualizer/adapters/edge_color.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/edge_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra_visualizer/color/color_parsing.h"

namespace hydra {
namespace {

#define REGISTER_COLOR_ADAPTER(adapter)                                           \
  static const auto adapter##registration =                                       \
      config::RegistrationWithConfig<EdgeColorAdapter, adapter, adapter::Config>( \
          #adapter)

REGISTER_COLOR_ADAPTER(UniformEdgeColorAdapter);
REGISTER_COLOR_ADAPTER(ValueEdgeColorAdapter);
REGISTER_COLOR_ADAPTER(TraversabilityEdgeColorAdapter);

#undef REGISTER_COLOR_ADAPTER

}  // namespace

using namespace spark_dsg;
using EdgeColor = EdgeColorAdapter::EdgeColor;

void declare_config(UniformEdgeColorAdapter::Config& config) {
  using namespace config;
  name("UniformEdgeColorAdapter::Config");
  field(config.color, "color");
}

UniformEdgeColorAdapter::UniformEdgeColorAdapter(const Config& config)
    : config(config) {}

EdgeColor UniformEdgeColorAdapter::getColor(const DynamicSceneGraph&,
                                            const SceneGraphEdge&) const {
  return {config.color, config.color};
}

double EdgeWeightFunctor::eval(const DynamicSceneGraph&,
                               const SceneGraphEdge& edge) const {
  return edge.attributes().weight;
}

ValueEdgeColorAdapter::ValueEdgeColorAdapter(const Config& config)
    : config(config),
      min_value_(0.0),
      max_value_(1.0),
      functor_(config::create<EdgeValueFunctor>(config.value_functor)),
      colormap_(config.colormap) {}

void ValueEdgeColorAdapter::setGraph(const DynamicSceneGraph& graph,
                                     LayerKey layer_key) {
  if (!functor_) {
    return;
  }

  bool is_first = true;
  try {
    const auto& layer = graph.getLayer(layer_key.layer, layer_key.partition);
    for (const auto& [key, edge] : layer.edges()) {
      const auto value = functor_->eval(graph, edge);
      if (is_first) {
        min_value_ = value;
        max_value_ = value;
        is_first = false;
      } else {
        min_value_ = std::min(value, min_value_);
        max_value_ = std::max(value, max_value_);
      }
    }
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
  }
}

EdgeColor ValueEdgeColorAdapter::getColor(const DynamicSceneGraph& graph,
                                          const SceneGraphEdge& edge) const {
  try {
    const auto color =
        colormap_.getColor(functor_->eval(graph, edge), min_value_, max_value_);
    return {color, color};
  } catch (const std::exception& e) {
    LOG_FIRST_N(ERROR, 1) << "Value functor unable to evaluate: " << e.what();
    return {Color(), Color()};
  }
}

void declare_config(ValueEdgeColorAdapter::Config& config) {
  using namespace config;
  name("ValueEdgeColorAdapter::Config");
  field(config.colormap, "colormap");
  field(config.value_functor, "value_functor");
}

TraversabilityEdgeColorAdapter::TraversabilityEdgeColorAdapter(const Config& config)
    : config(config), min_value_(0.0), max_value_(1.0), colormap_(config.colormap) {}

void TraversabilityEdgeColorAdapter::setGraph(const DynamicSceneGraph& graph,
                                              LayerKey key) {
  const auto layer = graph.findLayer(key.layer, key.partition);
  if (!layer) {
    return;
  }

  bool is_first = true;
  for (const auto& [key, edge] : layer->edges()) {
    const auto value = edge.attributes().weight;
    if (value < 0.0) {
      continue;
    }
    if (is_first) {
      min_value_ = value;
      max_value_ = value;
      is_first = false;
    } else {
      min_value_ = std::min(value, min_value_);
      max_value_ = std::max(value, max_value_);
    }
  }
}

EdgeColor TraversabilityEdgeColorAdapter::getColor(const DynamicSceneGraph&,
                                                   const SceneGraphEdge& edge) const {
  const double weight = edge.attributes().weight;
  if (weight == -1.0) {
    return {config.active_color, config.active_color};
  }
  if (weight == -2.0) {
    return {config.backend_color, config.backend_color};
  }
  if (weight < 0.0) {
    return {Color::gray(), Color::gray()};
  }
  const auto color = colormap_.getColor(weight, min_value_, max_value_);
  return {color, color};
}

void declare_config(TraversabilityEdgeColorAdapter::Config& config) {
  using namespace config;
  name("TraversabilityEdgeColorAdapter::Config");
  field(config.colormap, "colormap");
  field(config.active_color, "active_color");
  field(config.backend_color, "backend_color");
}

}  // namespace hydra
