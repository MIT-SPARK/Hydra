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
#include "hydra/common/global_info.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/labelspace.h>

#include "hydra/common/semantic_color_map.h"
#include "hydra/utils/pgmo_glog_sink.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using ColorMapPtr = std::shared_ptr<SemanticColorMap>;
using timing::ElapsedTimeRecorder;

decltype(GlobalInfo::instance_) GlobalInfo::instance_;

void declare_config(MeshFieldConfig& config) {
  using namespace config;
  name("MeshFieldConfig");
  field(config.with_colors, "with_colors");
  field(config.with_first_seen_stamps, "with_first_seen_stamps");
  field(config.with_labels, "with_labels");
}

void declare_config(FrameConfig& frames) {
  using namespace config;
  name("FrameConfig");
  field(frames.robot, "robot_frame");
  field(frames.odom, "odom_frame");
  field(frames.map, "map_frame");
}

void declare_config(PipelineConfig& config) {
  using namespace config;
  name("PipelineConfig");
  field(config.timing_disabled, "timing_disabled");
  field(config.disable_timer_output, "disable_timer_output");
  field(config.enable_pgmo_logging, "enable_pgmo_logging");
  field(config.default_verbosity, "default_verbosity");
  field(config.default_num_threads, "default_num_threads");
  field(config.store_visualization_details, "store_visualization_details");
  config.map_window.setOptional();
  field(config.map_window, "map_window");
  config.labelspace.setOptional();
  field(config.labelspace, "labelspace");
  field<Path::Absolute>(config.label_remap_filepath, "semantic_label_remap_filepath");
  field(config.mesh, "mesh");
  field(config.semantic_layers, "semantic_layers");

  // the following subconfigs should not be namespaced
  field(config.frames, "frames", false);
  field(config.graph, "graph", false);
}

GlobalInfo::GlobalInfo() : force_shutdown_(false) {}

void GlobalInfo::initFromConfig(const PipelineConfig& config, int robot_id) {
  config_ = config::checkValid(config);
  robot_prefix_ = RobotPrefixConfig(robot_id);

  auto& timer = ElapsedTimeRecorder::instance();
  timer.timing_disabled = config_.timing_disabled;
  timer.disable_output = config_.disable_timer_output;

  labelspace_ = config_.labelspace.create();
  if (!labelspace_) {
    labelspace_ = std::make_unique<Labelspace>();
  }

  if (!config_.label_remap_filepath.empty()) {
    label_remapper_ = LabelRemapper(config_.label_remap_filepath);
  }

  if (config_.enable_pgmo_logging) {
    logging::Logger::addSink("glog", std::make_shared<PgmoGlogSink>());
  }
}

GlobalInfo& GlobalInfo::instance() {
  if (!instance_) {
    instance_.reset(new GlobalInfo());
  }

  return *instance_;
}

GlobalInfo& GlobalInfo::init(const PipelineConfig& config, int robot_id) {
  auto& curr = instance();
  curr.initFromConfig(config, robot_id);
  return curr;
}

void GlobalInfo::reset() { instance_.reset(new GlobalInfo()); }

void GlobalInfo::setForceShutdown(bool force_shutdown) {
  force_shutdown_ = force_shutdown;
}

bool GlobalInfo::force_shutdown() const { return force_shutdown_; }

const PipelineConfig& GlobalInfo::getConfig() const { return config_; }

const FrameConfig& GlobalInfo::getFrames() const { return config_.frames; }

const RobotPrefixConfig& GlobalInfo::getRobotPrefix() const { return robot_prefix_; }

const Labelspace& GlobalInfo::labelspace() const {
  CHECK(labelspace_);
  return *labelspace_;
}

const LabelRemapper& GlobalInfo::getLabelRemapper() const { return label_remapper_; }

SharedDsgInfo::Ptr GlobalInfo::createSharedDsg() const {
  auto graph_info = std::make_shared<SharedDsgInfo>(config_.graph);
  auto& graph = *graph_info->graph;

  CHECK(labelspace_);
  const spark_dsg::Labelspace labelspace(labelspace_->label_names);
  if (labelspace) {
    labelspace.save(graph, "mesh", false);
    for (const auto& layer_name : config_.semantic_layers) {
      if (graph.layer_names().count(layer_name)) {
        labelspace.save(graph, layer_name);
      } else {
        LOG(WARNING) << "Skipping saving labelspace for unknown '" << layer_name << "'";
      }
    }
  }

  return graph_info;
}

bool GlobalInfo::setSensor(const Sensor::Ptr& sensor, bool allow_override) {
  if (!sensor) {
    LOG(ERROR) << "Sensor is invalid!";
    return false;
  }

  auto iter = sensors_.find(sensor->name);
  if (iter == sensors_.end()) {
    sensors_[sensor->name] = sensor;
    return true;
  }

  if (!allow_override) {
    LOG(ERROR) << "Sensor '" << sensor->name << "' already exists!";
    return false;
  }

  VLOG(1) << "Overriding sensor '" << sensor->name << "'!";
  iter->second = sensor;
  return true;
}

Sensor::ConstPtr GlobalInfo::getSensor(const std::string& name) const {
  auto iter = sensors_.find(name);
  if (iter == sensors_.end()) {
    LOG(ERROR) << "Sensor '" << name << "' does not exist!";
    return nullptr;
  }

  return iter->second;
}

std::vector<std::string> GlobalInfo::getAvailableSensors() const {
  std::vector<std::string> names;
  names.reserve(sensors_.size());
  for (const auto& [name, sensor] : sensors_) {
    names.push_back(name);
  }

  return names;
}

std::unique_ptr<VolumetricWindow> GlobalInfo::createVolumetricWindow() const {
  return config_.map_window.create();
}

spark_dsg::Mesh::Ptr GlobalInfo::createMesh() const {
  // we force the mesh to have last seen stamps
  return std::make_shared<spark_dsg::Mesh>(config_.mesh.with_colors,
                                           true,
                                           config_.mesh.with_labels,
                                           config_.mesh.with_first_seen_stamps);
}

std::ostream& operator<<(std::ostream& out, const GlobalInfo& config) {
  out << config::toString(config.getConfig());
  const auto sensor_names = config.getAvailableSensors();
  for (const auto& name : sensor_names) {
    auto sensor = config.getSensor(name);
    if (!sensor) {
      continue;
    }

    out << "sensor '" << name << "'" << sensor->dump();
  }

  return out;
}

}  // namespace hydra
