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

#include <config_utilities/dynamic_config.h>

#include "hydra_visualizer/plugins/layer_plugin.h"

namespace hydra {

class MeshPlacesPlugin : public LayerPlugin {
 public:
  //! @brief configuration for polygon boundaries
  struct Config {
    //! @brief display polygon boundaries
    bool draw = false;
    //! @brief draw polygons at mesh level
    bool collapse = false;
    //! @brief scale of boundary wireframe
    double wireframe_scale = 0.1;
    //! @brief draw polygons using node semantic color
    bool use_node_color = true;
    //! @brief alpha of boundary
    double alpha = 0.5;
    //! @brief display minimum bounding ellipse
    bool draw_ellipse = false;
    //! @brief alpha of bounding ellipse
    double ellipse_alpha = 0.5;
  };

  MeshPlacesPlugin(const Config& config, const std::string& ns);

  virtual ~MeshPlacesPlugin() = default;

  virtual void draw(const std_msgs::msg::Header& header,
                    const visualizer::LayerInfo& info,
                    const spark_dsg::SceneGraphLayer& layer,
                    const spark_dsg::Mesh* mesh,
                    visualization_msgs::msg::MarkerArray& msg,
                    MarkerTracker& tracker) override;

  YAML::Node dumpConfig() const override;

 protected:
  const std::string ns_;
  config::DynamicConfig<Config> config_;
};

void declare_config(MeshPlacesPlugin::Config& config);

}  // namespace hydra
