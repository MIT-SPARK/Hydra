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
#include "hydra/active_window/reconstruction_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

#include <chrono>

#include "hydra/common/global_info.h"
#include "hydra/input/input_conversion.h"
#include "hydra/places/robot_footprint_integrator.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<ActiveWindowModule,
                                   ReconstructionModule,
                                   ReconstructionModule::Config,
                                   ActiveWindowModule::OutputQueue::Ptr>(
        "ReconstructionModule");

double diffInSeconds(uint64_t lhs, uint64_t rhs) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(
             std::chrono::nanoseconds(lhs) - std::chrono::nanoseconds(rhs))
      .count();
}

}  // namespace

using timing::ScopedTimer;

void declare_config(ReconstructionModule::Config& config) {
  using namespace config;
  name("ReconstructionModule::Config");
  base<ActiveWindowModule::Config>(config);
  field(config.full_update_separation_s, "full_update_separation_s", "s");
  field(config.max_input_queue_size, "max_input_queue_size");
  field(config.tsdf, "tsdf");
  field(config.mesh, "mesh");
  config.robot_footprint.setOptional();
  field(config.robot_footprint, "robot_footprint");
}

ReconstructionModule::ReconstructionModule(const Config& config,
                                           const OutputQueue::Ptr& queue)
    : ActiveWindowModule(config, queue),
      config(config::checkValid(config)),
      last_update_ns_(std::nullopt),
      tsdf_integrator_(std::make_unique<ProjectiveIntegrator>(config.tsdf)),
      mesh_integrator_(std::make_unique<MeshIntegrator>(config.mesh)),
      footprint_integrator_(config.robot_footprint.create()) {
  if (config.tsdf.semantic_integrator && !map_.config.with_semantics) {
    LOG(ERROR)
        << "Semantic integrator specified but map does not contain semantic layer!";
  }
}

ReconstructionModule::~ReconstructionModule() {}

std::string ReconstructionModule::printInfo() const {
  return config::toString(config) + "\n" + Sink::printSinks(sinks_);
}

bool ReconstructionModule::shouldUpdate(uint64_t timestamp_ns) const {
  if (!last_update_ns_) {
    return true;
  }

  const auto diff_s = diffInSeconds(timestamp_ns, last_update_ns_.value());
  return diff_s >= config.full_update_separation_s;
}

ActiveWindowOutput::Ptr ReconstructionModule::spinOnce(const InputPacket& msg) {
  if (!msg.sensor_input) {
    LOG(ERROR) << "[Hydra Reconstruction] received invalid sensor data in input!";
    return nullptr;
  }

  const auto timestamp_ns = msg.timestamp_ns;
  const auto world_T_body = msg.world_T_body();
  const auto do_full_update = shouldUpdate(timestamp_ns);

  VLOG(2) << "[Hydra Reconstruction] starting " << (do_full_update ? "full" : "partial")
          << " update for message @ " << timestamp_ns << " (" << input_queue_->size()
          << " message(s) left)";

  ScopedTimer timer("reconstruction/spin", timestamp_ns);
  InputData::Ptr data = conversions::parseInputPacket(msg);
  if (!data) {
    return nullptr;
  }

  {  // timing scope
    ScopedTimer timer("reconstruction/tsdf", timestamp_ns);
    tsdf_integrator_->updateMap(*data, map_);
    if (footprint_integrator_) {
      footprint_integrator_->markFreespace(world_T_body.cast<float>(), map_);
    }
  }  // timing scope

  auto& tsdf = map_.getTsdfLayer();
  if (tsdf.numBlocks() == 0 || !do_full_update) {
    return nullptr;
  }

  {  // timing scope
    ScopedTimer timer("reconstruction/mesh", timestamp_ns);
    mesh_integrator_->generateMesh(map_, true, true);
  }  // timing scope

  auto output = ActiveWindowOutput::fromInput(msg);
  output->sensor_data = data;

  // this comes before clearing the update flag as we don't archive updated blocks
  if (map_window_) {
    output->archived_mesh_indices =
        map_window_->archiveBlocks(timestamp_ns, world_T_body, map_);
    VLOG(2) << "[Hydra Reconstruction] archived "
            << output->archived_mesh_indices.size() << " @ " << timestamp_ns << " [ns]";
  }

  output->setMap(map_.cloneUpdated());
  for (const auto& block : tsdf) {
    block.clearUpdated();
  }

  return output;
}

}  // namespace hydra
