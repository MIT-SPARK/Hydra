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
#include "hydra/reconstruction/reconstruction_module.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/conversions.h>
#include <config_utilities/types/eigen_matrix.h>
#include <config_utilities/validation.h>

#include <chrono>

#include "hydra/common/global_info.h"
#include "hydra/input/input_conversion.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using timing::ScopedTimer;

namespace {

double diffInSeconds(uint64_t lhs, uint64_t rhs) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(
             std::chrono::nanoseconds(lhs) - std::chrono::nanoseconds(rhs))
      .count();
}

}  // namespace

void declare_config(ReconstructionModule::Config& config) {
  using namespace config;
  name("ReconstructionConfig");
  field(config.show_stats, "show_stats");
  field(config.stats_verbosity, "stats_verbosity");
  field(config.full_update_separation_s, "full_update_separation_s", "s");
  field(config.max_input_queue_size, "max_input_queue_size");
  config.map_window.setOptional();
  field(config.map_window, "map_window");
  field(config.tsdf, "tsdf");
  field(config.mesh, "mesh");
  config.robot_footprint.setOptional();
  field(config.robot_footprint, "robot_footprint");
  field(config.sinks, "sinks");
}

ReconstructionModule::ReconstructionModule(const Config& config,
                                           const OutputQueue::Ptr& queue)
    : config(config::checkValid(config)),
      queue_(new InputPacketQueue(config.max_input_queue_size)),
      last_update_ns_(std::nullopt),
      output_queue_(queue),
      sinks_(Sink::instantiate(config.sinks)),
      map_(new VolumetricMap(GlobalInfo::instance().getMapConfig(), true)),
      map_window_(config.map_window ? config.map_window.create()
                                    : GlobalInfo::instance().createVolumetricWindow()),
      tsdf_integrator_(std::make_unique<ProjectiveIntegrator>(config.tsdf)),
      mesh_integrator_(std::make_unique<MeshIntegrator>(config.mesh)),
      footprint_integrator_(config.robot_footprint.create()) {}

ReconstructionModule::~ReconstructionModule() { stopImpl(); }

void ReconstructionModule::start() {
  spin_thread_.reset(new std::thread(&ReconstructionModule::spin, this));
  LOG(INFO) << "[Hydra Reconstruction] started!";
}

void ReconstructionModule::stop() { stopImpl(); }

void ReconstructionModule::stopImpl() {
  should_shutdown_ = true;

  if (spin_thread_) {
    VLOG(2) << "[Hydra Reconstruction] stopping reconstruction!";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[Hydra Reconstruction] stopped!";
  }

  VLOG(2) << "[Hydra Reconstruction] input queue: " << queue_->size();
  if (output_queue_) {
    VLOG(2) << "[Hydra Reconstruction] output queue: " << output_queue_->size();
  } else {
    VLOG(2) << "[Hydra Reconstruction] output queue: n/a";
  }
}

void ReconstructionModule::save(const LogSetup&) {}

std::string ReconstructionModule::printInfo() const {
  std::stringstream ss;
  ss << std::endl << config::toString(config);
  return ss.str();
}

void ReconstructionModule::spin() {
  // TODO(nathan) fix shutdown logic
  while (!should_shutdown_) {
    bool has_data = queue_->poll();
    if (!has_data) {
      continue;
    }

    spinOnce(*queue_->front());
    queue_->pop();
  }
}

bool ReconstructionModule::spinOnce() {
  bool has_data = queue_->poll();
  if (!has_data) {
    return false;
  }

  const auto success = spinOnce(*queue_->front());
  queue_->pop();
  return success;
}

bool ReconstructionModule::spinOnce(const InputPacket& msg) {
  if (!msg.sensor_input) {
    LOG(ERROR) << "[Hydra Reconstruction] received invalid sensor data in input!";
    return false;
  }

  ScopedTimer timer("places/spin", msg.timestamp_ns);
  VLOG(2) << "[Hydra Reconstruction]: Processing msg @ " << msg.timestamp_ns;
  VLOG(2) << "[Hydra Reconstruction]: " << queue_->size() << " message(s) left";

  const bool do_full_update =
      !last_update_ns_ || diffInSeconds(msg.timestamp_ns, last_update_ns_.value()) >=
                              config.full_update_separation_s;
  update(msg, do_full_update);
  return do_full_update;
}

void ReconstructionModule::addSink(const Sink::Ptr& sink) {
  if (sink) {
    sinks_.push_back(sink);
  }
}

void ReconstructionModule::fillOutput(ReconstructionOutput& msg) {
  // TODO(nathan) figure out a better way to handle repeated timestamps
  size_t ts = msg.timestamp_ns;
  while (timestamp_cache_.count(ts)) {
    ++ts;
  }

  timestamp_cache_.insert(ts);
  msg.timestamp_ns = ts;
  msg.setMap(map_->cloneUpdated());
}

bool ReconstructionModule::update(const InputPacket& msg, bool full_update) {
  VLOG(2) << "[Hydra Reconstruction] starting " << ((full_update) ? "full" : "partial")
          << " update @ " << msg.timestamp_ns << " [ns]";

  InputData::Ptr data = conversions::parseInputPacket(msg);
  if (!data) {
    return false;
  }

  {  // timing scope
    ScopedTimer timer("places/tsdf", msg.timestamp_ns);
    tsdf_integrator_->updateMap(*data, *map_);
  }  // timing scope

  if (footprint_integrator_) {
    footprint_integrator_->addFreespaceFootprint(msg.world_T_body().cast<float>(),
                                                 *map_);
  }

  auto& tsdf = map_->getTsdfLayer();
  if (tsdf.numBlocks() == 0 || !full_update) {
    return false;
  }

  {  // timing scope
    ScopedTimer timer("places/mesh", msg.timestamp_ns);
    mesh_integrator_->generateMesh(*map_, true, true);
  }  // timing scope

  if (config.show_stats) {
    VLOG(config.stats_verbosity) << "Memory used: {" << map_->printStats() << "}";
  }

  auto output = ReconstructionOutput::fromInput(msg);
  output->sensor_data = data;
  fillOutput(*output);

  Sink::callAll(sinks_, msg.timestamp_ns, data->getSensorPose(), tsdf, *output);
  if (output_queue_) {
    output_queue_->push(output);
  }

  // NOTE(nathan) this comes before clearing the update flag because we don't archive
  // blocks that were just updated
  if (map_window_) {
    map_window_->archiveBlocks(msg.timestamp_ns, msg.world_T_body(), *map_);
  }

  for (const auto& block : tsdf) {
    block.clearUpdated();
  }

  return true;
}

}  // namespace hydra
