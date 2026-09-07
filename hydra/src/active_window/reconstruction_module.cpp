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
#include <glog/logging.h>

#include <chrono>
#include <iomanip>

#include "hydra/places/robot_footprint_integrator.h"
#include "hydra/utils/printing.h"
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

std::string printRotation(const Eigen::Matrix3d& rot) {
  const Eigen::Quaterniond q(rot);
  std::stringstream ss;
  ss << std::setprecision(3) << "{w: " << q.w() << ", x: " << q.x() << ", y: " << q.y()
     << ", z: " << q.z() << "}";
  return ss.str();
}

}  // namespace

using timing::ScopedTimer;

void declare_config(ReconstructionModule::Config& config) {
  using namespace config;
  name("ReconstructionModule::Config");
  base<ActiveWindowModule::Config>(config);
  field(config.full_update_separation_s, "full_update_separation_s", "s");
  field(config.mesh, "mesh");
  field(config.tsdf, "tsdf");
  config.robot_footprint.setOptional();
  field(config.robot_footprint, "robot_footprint");
}

ReconstructionModule::ReconstructionModule(const Config& config,
                                           const OutputQueue::Ptr& queue)
    : ActiveWindowModule(config, queue),
      config(config::checkValid(config)),
      last_update_ns_(std::nullopt),
      tsdf_integrators_(config.tsdf),
      mesh_integrator_(std::make_unique<MeshIntegrator>(config.mesh)),
      footprint_integrator_(config.robot_footprint.create()) {
  for (const auto& [name, tsdf_config] : config.tsdf.sensors) {
    if (tsdf_config.semantic_integrator && !map_.config.with_semantics) {
      LOG(ERROR) << "Semantic integrator specified for sensor " << name
                 << " but map does not contain semantic layer!";
    }
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

ActiveWindowOutput::Ptr ReconstructionModule::spinOnce(const InputData::Ptr& data) {
  if (!data) {
    LOG(ERROR) << "[active_window] received invalid input!";
    return nullptr;
  }

  const auto stamp = data->timestamp_ns;
  const auto fmt = getDefaultFormat();
  MLOG(3) << "Got input @ " << stamp
          << " [ns] with pose: p=" << data->world_T_body.translation().format(fmt)
          << ", q=" << printRotation(data->world_T_body.rotation());

  const auto do_full_update = shouldUpdate(stamp);
  MLOG(2) << "starting " << (do_full_update ? "full" : "partial")
          << " update for message @ " << stamp << " (" << input_queue_->size()
          << " message(s) left)";

  ScopedTimer timer("reconstruction/spin", stamp);
  // force semantic normalization if volumetric map has semantic layer
  if (!data->finalize(false, map_.hasSemantics())) {
    return nullptr;
  }

  const auto tsdf_integrator = tsdf_integrators_.get(data->getSensor().name);
  if (!tsdf_integrator) {
    MLOG(1) << "Unknown sensor '" << data->getSensor().name << "'";
    return nullptr;
  }

  {  // timing scope
    ScopedTimer timer("reconstruction/tsdf", stamp);
    const auto integration_mask = getDefaultIntegrationMask(*data);
    tsdf_integrator->updateMap(*data, map_, true, integration_mask);
    if (footprint_integrator_) {
      footprint_integrator_->markFreespace(data->world_T_body.cast<float>(), map_);
    }
  }  // timing scope

  auto& tsdf = map_.getTsdfLayer();
  if (tsdf.numBlocks() == 0 || !do_full_update) {
    return nullptr;
  }

  last_update_ns_ = stamp;
  {  // timing scope
    ScopedTimer timer("reconstruction/mesh", stamp);
    mesh_integrator_->generateMesh(map_, true, true);
  }  // timing scope

  auto output = std::make_shared<ActiveWindowOutput>();
  output->timestamp_ns = data->timestamp_ns;
  output->sensor_data = data;

  if (map_window_) {
    // this comes before clearing the update flag as we don't archive updated blocks
    output->archived = map_window_->archiveBlocks(stamp, data->world_T_body, map_);
    MLOG(2) << "archived " << output->archived.size() << " @ " << stamp << " [ns]";
  }

  output->setMap(map_.cloneUpdated());
  for (const auto& block : tsdf) {
    block.clearUpdated();
  }

  return output;
}

}  // namespace hydra
