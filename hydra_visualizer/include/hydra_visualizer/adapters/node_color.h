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

struct NodeColorAdapter {
  using Ptr = std::shared_ptr<NodeColorAdapter>;

  virtual ~NodeColorAdapter() = default;

  /**
   * @brief Get color for a single node
   * @param graph Current scene graph node is from
   * @param node Node to get color for
   * @returns Visualizer color for node
   */
  virtual spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                                    const spark_dsg::SceneGraphNode& node) const = 0;

  /**
   * @brief Set any pre-draw information
   * @param graph Graph to get information for
   * @param layer Layer to get information for
   *
   * Allows color adapters to gather statistics about the scene graph before generating
   * any node colors when drawing the scene graph
   */
  virtual void setGraph(const spark_dsg::DynamicSceneGraph& /* graph */,
                        spark_dsg::LayerKey /* layer */) {}
};

struct AttributeColorAdapter : NodeColorAdapter {
  struct Config {
    spark_dsg::Color default_color;
  } const config;

  explicit AttributeColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(AttributeColorAdapter::Config& config);

struct UniformColorAdapter : NodeColorAdapter {
  struct Config {
    spark_dsg::Color color;
  } const config;

  explicit UniformColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(UniformColorAdapter::Config& config);

struct LabelColorAdapter : NodeColorAdapter {
  struct Config {
    visualizer::CategoricalColormap::Config colormap;
  } const config;

  explicit LabelColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::CategoricalColormap colormap_;
};

void declare_config(LabelColorAdapter::Config& config);

struct IdColorAdapter : NodeColorAdapter {
  struct Config {
    visualizer::DiscreteColormap::Config colormap;
  } const config;

  explicit IdColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::DiscreteColormap colormap_;
};

void declare_config(IdColorAdapter::Config& config);

struct ParentColorAdapter : NodeColorAdapter {
  struct Config {
    spark_dsg::Color default_color;
    config::VirtualConfig<NodeColorAdapter> parent_adapter{
        IdColorAdapter::Config{config::VirtualConfig<visualizer::DiscretePalette>{
            visualizer::ColorbrewerPalette::Config{}}}};
  } const config;

  explicit ParentColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const NodeColorAdapter::Ptr parent_adapter_;
};

void declare_config(ParentColorAdapter::Config& config);

struct StatusFunctor {
  virtual ~StatusFunctor() = default;
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const = 0;
};

struct IsActiveFunctor : StatusFunctor {
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const override;
};

struct HasActiveMeshFunctor : StatusFunctor {
  virtual bool eval(const spark_dsg::DynamicSceneGraph& graph,
                    const spark_dsg::SceneGraphNode& node) const override;
};

struct StatusColorAdapter : NodeColorAdapter {
  struct Config {
    spark_dsg::Color true_color{0, 255, 0};
    spark_dsg::Color false_color;
    std::string status_functor{"is_active"};
  } const config;

  explicit StatusColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  std::unique_ptr<StatusFunctor> functor_;
};

void declare_config(StatusColorAdapter::Config& config);

struct FrontierColorAdapter : NodeColorAdapter {
  struct Config {
    spark_dsg::Color real;
    spark_dsg::Color predicted{0, 0, 255};
    spark_dsg::Color active{0, 255, 0};
    spark_dsg::Color archived{255, 0, 0};
    spark_dsg::Color anti_frontier{0, 255, 255};
  } const config;

  explicit FrontierColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(FrontierColorAdapter::Config& config);

struct PartitionColorAdapter : NodeColorAdapter {
  struct Config {
    visualizer::DiscreteColormap::Config colormap;
  } const config;

  explicit PartitionColorAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::DiscreteColormap colormap_;
};

void declare_config(PartitionColorAdapter::Config& config);

struct ValueFunctor {
  virtual ~ValueFunctor() = default;
  virtual double eval(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const = 0;
};

struct DistanceFunctor : ValueFunctor {
  double eval(const spark_dsg::DynamicSceneGraph& graph,
              const spark_dsg::SceneGraphNode& node) const override;
};

struct LastUpdatedFunctor : ValueFunctor {
  double eval(const spark_dsg::DynamicSceneGraph& graph,
              const spark_dsg::SceneGraphNode& node) const override;
};

struct ValueColorAdapter : NodeColorAdapter {
  struct Config {
    visualizer::RangeColormap::Config colormap;
    std::string value_functor{"place_distance"};
  } const config;

  explicit ValueColorAdapter(const Config& config);
  void setGraph(const spark_dsg::DynamicSceneGraph& graph,
                spark_dsg::LayerKey layer) override;
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  double min_value_;
  double max_value_;
  std::unique_ptr<ValueFunctor> functor_;
  const visualizer::RangeColormap colormap_;
};

void declare_config(ValueColorAdapter::Config& config);

struct LabelDistributionAdapter : NodeColorAdapter {
  struct Config {
    visualizer::CategoricalColormap::Config colormap;
  } const config;

  explicit LabelDistributionAdapter(const Config& config);
  spark_dsg::Color getColor(const spark_dsg::DynamicSceneGraph& graph,
                            const spark_dsg::SceneGraphNode& node) const override;

 private:
  const visualizer::CategoricalColormap colormap_;
};

void declare_config(LabelDistributionAdapter::Config& config);

}  // namespace hydra
