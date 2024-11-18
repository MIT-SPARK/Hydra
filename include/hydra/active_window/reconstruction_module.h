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

#include <Eigen/Geometry>

#include "hydra/active_window/active_window_module.h"
#include "hydra/reconstruction/mesh_integrator_config.h"
#include "hydra/reconstruction/projective_integrator.h"

namespace hydra {

class ProjectiveIntegrator;
class MeshIntegrator;
class RobotFootprintIntegrator;

class ReconstructionModule : public ActiveWindowModule {
 public:
  struct Config : ActiveWindowModule::Config {
    double full_update_separation_s = 0.0;
    ProjectiveIntegrator::Config tsdf;
    MeshIntegratorConfig mesh;
    config::VirtualConfig<RobotFootprintIntegrator> robot_footprint;
  } const config;

  ReconstructionModule(const Config& config, const OutputQueue::Ptr& output_queue);

  virtual ~ReconstructionModule();

  std::string printInfo() const override;

 protected:
  bool shouldUpdate(uint64_t timestamp_ns) const;

  ActiveWindowOutput::Ptr spinOnce(const InputPacket& input) override;

 protected:
  std::optional<uint64_t> last_update_ns_;

  std::unique_ptr<ProjectiveIntegrator> tsdf_integrator_;
  std::unique_ptr<MeshIntegrator> mesh_integrator_;
  std::unique_ptr<RobotFootprintIntegrator> footprint_integrator_;
};

void declare_config(ReconstructionModule::Config& config);

}  // namespace hydra
