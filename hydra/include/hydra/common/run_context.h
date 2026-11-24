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

#include <memory>
#include <vector>

#include "hydra/common/labelspace.h"
#include "hydra/common/robot_config.h"
#include "hydra/common/run_settings.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/utils/data_directory.h"

namespace hydra {

class RunContext {
 public:
  using ConstPtr = std::shared_ptr<const RunContext>;

  struct Config {
    RobotConfig robot;
    RunSettings::Config settings = RunSettings::instance();
    //! Base artifact directory; an empty path disables run logging
    std::filesystem::path log_path;
    //! Directory allocation and overwrite settings
    DataDirectory::Config output;
    //! Layer names for the scene graph
    SharedDsgInfo::Config graph;
    //! Graph layers whose labels use the run labelspace
    std::vector<std::string> semantic_layers{spark_dsg::DsgLayers::OBJECTS,
                                             spark_dsg::DsgLayers::MESH_PLACES};
    //! Source of semantic labels, resolved when constructing the context
    config::VirtualConfig<Labelspace> labelspace{Labelspace{}};
  };

  explicit RunContext(const Config& config);
  RunContext(const RunContext&) = delete;
  RunContext& operator=(const RunContext&) = delete;

  const Config config;
  const DataDirectory logs;

  RobotPrefixConfig robot_prefix() const;
  const Labelspace& labelspace() const;

 private:
  std::unique_ptr<Labelspace> labelspace_;
};

void declare_config(RunContext::Config& config);

}  // namespace hydra
