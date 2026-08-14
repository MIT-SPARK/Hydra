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
#include <config_utilities/factory.h>
#include <spark_dsg/color.h>
#include <spark_dsg/dynamic_scene_graph.h>

#include <memory>

#include "hydra_visualizer/color/colormap_utilities.h"

namespace hydra {

struct EdgeColorAdapter {
  using Ptr = std::shared_ptr<EdgeColorAdapter>;
  using EdgeColor = std::pair<spark_dsg::Color, spark_dsg::Color>;

  virtual ~EdgeColorAdapter() = default;

  /**
   * @brief Get color for an edge.
   * @param graph Current scene graph node is from
   * @param edge Edge to get color for
   * @returns Visualizer color for source and target ends of the edge.
   */
  virtual EdgeColor getColor(const spark_dsg::DynamicSceneGraph& graph,
                             const spark_dsg::SceneGraphEdge& edge) const = 0;

  /**
   * @brief Set any pre-draw information
   * @param graph Graph to get information for
   *
   * Allows color adapters to gather statistics about the scene graph before generating
   * any edge colors when drawing the scene graph
   */
  virtual void setGraph(const spark_dsg::DynamicSceneGraph& /* graph */,
                        spark_dsg::LayerKey /* layer */) {}
};

struct UniformEdgeColorAdapter : EdgeColorAdapter {
  struct Config {
    // TODO(lschmid): Consider using the named colors, or even better integrate
    // optionally named colors directly into the config utilities parsing.
    spark_dsg::Color color;
  } const config;

  explicit UniformEdgeColorAdapter(const Config& config);
  EdgeColor getColor(const spark_dsg::DynamicSceneGraph& graph,
                     const spark_dsg::SceneGraphEdge& edge) const override;
};

void declare_config(UniformEdgeColorAdapter::Config& config);

struct EdgeValueFunctor {
  virtual ~EdgeValueFunctor() = default;
  virtual double eval(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphEdge& edge) const = 0;
};

struct EdgeWeightFunctor : EdgeValueFunctor {
  double eval(const spark_dsg::DynamicSceneGraph& graph,
              const spark_dsg::SceneGraphEdge& edge) const override;

  inline static const auto registration =
      config::Registration<EdgeValueFunctor, EdgeWeightFunctor>("weight");
};

struct ValueEdgeColorAdapter : EdgeColorAdapter {
  struct Config {
    visualizer::RangeColormap::Config colormap;
    std::string value_functor{"weight"};
  } const config;

  explicit ValueEdgeColorAdapter(const Config& config);
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerKey layer) override;
  EdgeColor getColor(const spark_dsg::DynamicSceneGraph& graph,
                     const spark_dsg::SceneGraphEdge& edge) const override;

 private:
  double min_value_;
  double max_value_;
  std::unique_ptr<EdgeValueFunctor> functor_;
  const visualizer::RangeColormap colormap_;
};

void declare_config(ValueEdgeColorAdapter::Config& config);

struct TraversabilityEdgeColorAdapter : EdgeColorAdapter {
  struct Config {
    visualizer::RangeColormap::Config colormap{
        config::VirtualConfig<visualizer::ContinuousPalette>{
            visualizer::QualityPalette::Config()}};
    spark_dsg::Color active_color = spark_dsg::Color::pink();
    spark_dsg::Color backend_color = spark_dsg::Color::blue();
  } const config;

  explicit TraversabilityEdgeColorAdapter(const Config& config);
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerKey layer) override;
  EdgeColor getColor(const spark_dsg::DynamicSceneGraph& graph,
                     const spark_dsg::SceneGraphEdge& edge) const override;

 private:
  double min_value_;
  double max_value_;
  const visualizer::RangeColormap colormap_;
};

void declare_config(TraversabilityEdgeColorAdapter::Config& config);

}  // namespace hydra
