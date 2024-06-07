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
#include <pose_graph_tools_ros/conversions.h>

#include "hydra/common/hydra_config.h"
#include "hydra/reconstruction/mesh_integrator.h"
#include "hydra/reconstruction/projective_integrator.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using pose_graph_tools_msgs::PoseGraph;
using timing::ScopedTimer;
using voxblox::BlockIndexList;
using voxblox::Layer;

void declare_config(ReconstructionModule::Config& conf) {
  using namespace config;
  name("ReconstructionConfig");
  field(conf.show_stats, "show_stats");
  field(conf.stats_verbosity, "stats_verbosity");
  field(conf.clear_distant_blocks, "clear_distant_blocks");
  field(conf.dense_representation_radius_m, "dense_representation_radius_m");
  field(conf.num_poses_per_update, "num_poses_per_update");
  field(conf.max_input_queue_size, "max_input_queue_size");
  field(conf.semantic_measurement_probability, "semantic_measurement_probability");
  field(conf.tsdf, "tsdf");
  field(conf.mesh, "mesh");
  field(conf.pose_graphs, "pose_graphs");
  conf.robot_footprint.setOptional();
  field(conf.robot_footprint, "robot_footprint");
  field(conf.sinks, "sinks");
}

ReconstructionModule::ReconstructionModule(const Config& config,
                                           const OutputQueue::Ptr& queue)
    : config(config::checkValid(config)),
      num_poses_received_(0),
      pose_graph_tracker_(new PoseGraphTracker(config.pose_graphs)),
      output_queue_(queue),
      sinks_(Sink::instantiate(config.sinks)) {
  queue_.reset(new ReconstructionInputQueue());
  queue_->max_size = config.max_input_queue_size;

  map_.reset(new VolumetricMap(HydraConfig::instance().getMapConfig(), true, true));
  tsdf_integrator_ = std::make_unique<ProjectiveIntegrator>(config.tsdf);
  mesh_integrator_ = std::make_unique<MeshIntegrator>(config.mesh);
  footprint_integrator_ = config.robot_footprint.create();
}

ReconstructionModule::~ReconstructionModule() { stop(); }

void ReconstructionModule::start() {
  spin_thread_.reset(new std::thread(&ReconstructionModule::spin, this));
  LOG(INFO) << "[Hydra Reconstruction] started!";
}

void ReconstructionModule::stop() {
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

bool ReconstructionModule::spinOnce(const ReconstructionInput& msg) {
  if (!msg.sensor_input) {
    LOG(ERROR) << "[Hydra Reconstruction] received invalid sensor data in input!";
    return false;
  }

  ScopedTimer timer("places/spin", msg.timestamp_ns);
  VLOG(2) << "[Hydra Reconstruction]: Processing msg @ " << msg.timestamp_ns;
  VLOG(2) << "[Hydra Reconstruction]: " << queue_->size() << " message(s) left";

  pose_graph_tracker_->update(msg);
  if (msg.agent_node_measurements) {
    agent_node_measurements_ = msg.agent_node_measurements;
  }

  ++num_poses_received_;
  const bool do_full_update = (num_poses_received_ % config.num_poses_per_update == 0);
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

  pose_graph_tracker_->fillPoseGraphs(msg);
  if (agent_node_measurements_) {
    msg.agent_node_measurements = agent_node_measurements_;
    agent_node_measurements_.reset();
  }

  msg.setMap(*map_);

  if (!config.clear_distant_blocks) {
    return;
  }

  const auto indices = findBlocksToArchive(msg.world_t_body.cast<float>());
  msg.archived_blocks.insert(msg.archived_blocks.end(), indices.begin(), indices.end());
  map_->removeBlocks(indices);
}

bool ReconstructionModule::update(const ReconstructionInput& msg, bool full_update) {
  VLOG(2) << "[Hydra Reconstruction] starting " << ((full_update) ? "full" : "partial")
          << " update @ " << msg.timestamp_ns << " [ns]";

  auto data = std::make_shared<FrameData>();
  if (!msg.fillFrameData(*data)) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to construct valid input packet!";
    return false;
  }

  if (!data->normalizeData()) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to convert all data";
    return false;
  }

  const Sensor& sensor = *HydraConfig::instance().getSensor(msg.sensor_input->sensor_id);
  if (!sensor.finalizeRepresentations(*data)) {
    LOG(ERROR) << "[Hydra Reconstruction] unable to compute inputs for integration";
    return false;
  }

  {  // timing scope
    ScopedTimer timer("places/tsdf", msg.timestamp_ns);
    tsdf_integrator_->updateMap(sensor, *data, *map_);
  }  // timing scope

  if (footprint_integrator_) {
    footprint_integrator_->addFreespaceFootprint(msg.world_T_body<float>(), *map_);
  }

  if (map_->getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
    return false;
  }

  {  // timing scope
    ScopedTimer timer("places/mesh", msg.timestamp_ns);
    mesh_integrator_->generateMesh(*map_, true, true);
  }  // timing scope

  if (!full_update) {
    return false;
  }

  if (config.show_stats) {
    VLOG(config.stats_verbosity) << "Memory used: {" << map_->printStats() << "}";
  }

  auto output = ReconstructionOutput::fromInput(msg);
  output->sensor_data = data;
  fillOutput(*output);

  Sink::callAll(sinks_,
                msg.timestamp_ns,
                data->getSensorPose(sensor),
                map_->getTsdfLayer(),
                *output);

  if (output_queue_) {
    output_queue_->push(output);
  }

  auto& tsdf = map_->getTsdfLayer();
  auto& mesh = map_->getMeshLayer();
  BlockIndexList blocks;
  tsdf.getAllUpdatedBlocks(voxblox::Update::kEsdf, &blocks);
  for (const auto& idx : blocks) {
    tsdf.getBlockByIndex(idx).updated().reset(voxblox::Update::kEsdf);
    mesh.getMeshBlock(idx)->updated = false;
  }

  return true;
}

// TODO(nathan) push to map?
BlockIndexList ReconstructionModule::findBlocksToArchive(
    const Eigen::Vector3f& center) const {
  const auto& tsdf = map_->getTsdfLayer();
  BlockIndexList blocks;
  tsdf.getAllAllocatedBlocks(&blocks);

  BlockIndexList to_archive;
  for (const auto& idx : blocks) {
    auto block = tsdf.getBlockPtrByIndex(idx);
    if ((center - block->origin()).norm() < config.dense_representation_radius_m) {
      continue;
    }

    // TODO(nathan) filter by update flag?
    to_archive.push_back(idx);
  }

  return to_archive;
}

}  // namespace hydra
