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
#include <spark_dsg/dynamic_scene_graph.h>
#include <spark_dsg/labelspace.h>

namespace hydra::visualizer {

struct NodeTextAdapter {
  using Ptr = std::shared_ptr<NodeTextAdapter>;
  virtual ~NodeTextAdapter() = default;
  virtual std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                              const spark_dsg::SceneGraphNode& node) const = 0;
};

struct IdTextAdapter : NodeTextAdapter {
  struct Config {};
  explicit IdTextAdapter(const Config&) {}
  virtual ~IdTextAdapter() = default;
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(IdTextAdapter::Config& config);

struct LabelTextAdapter : NodeTextAdapter {
  struct Config {};
  explicit LabelTextAdapter(const Config&) {}
  virtual ~LabelTextAdapter() = default;
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;

  mutable std::map<std::string, spark_dsg::Labelspace> labelspaces_;
};

void declare_config(LabelTextAdapter::Config& config);

struct LabelIdTextAdapter : NodeTextAdapter {
  struct Config {};
  explicit LabelIdTextAdapter(const Config&) {}
  virtual ~LabelIdTextAdapter() = default;
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;

  mutable std::map<std::string, spark_dsg::Labelspace> labelspaces_;
};

void declare_config(LabelIdTextAdapter::Config& config);

struct NameTextAdapter : NodeTextAdapter {
  struct Config {};
  explicit NameTextAdapter(const Config&) {}
  virtual ~NameTextAdapter() = default;
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(NameTextAdapter::Config& config);

struct NameIdTextAdapter : NodeTextAdapter {
  struct Config {};
  explicit NameIdTextAdapter(const Config&) {}
  virtual ~NameIdTextAdapter() = default;
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(NameIdTextAdapter::Config& config);

struct AttributesTextAdaptor : NodeTextAdapter {
  struct Config {};
  explicit AttributesTextAdaptor(const Config&) {}
  virtual ~AttributesTextAdaptor() = default;
  std::string getText(const spark_dsg::DynamicSceneGraph& graph,
                      const spark_dsg::SceneGraphNode& node) const override;
};

void declare_config(AttributesTextAdaptor::Config& config);

}  // namespace hydra::visualizer
