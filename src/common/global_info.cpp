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
#include <config_utilities/validation.h>
#include <spark_dsg/labelspace.h>

#include <filesystem>
#include <fstream>

#include "hydra/common/config_utilities.h"
#include "hydra/common/semantic_color_map.h"
#include "hydra/utils/pgmo_glog_sink.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using ColorMapPtr = std::shared_ptr<SemanticColorMap>;
using timing::ElapsedTimeRecorder;

decltype(GlobalInfo::instance_) GlobalInfo::instance_;

struct LabelNameConversion {
  using YamlList = std::vector<std::map<std::string, std::string>>;
  using SourceMap = std::map<uint32_t, std::string>;

  static YamlList toIntermediate(const SourceMap& other, std::string&) {
    YamlList to_return;
    for (const auto& kv_pair : other) {
      std::map<std::string, std::string> value_map{
          {"label", std::to_string(kv_pair.first)}, {"name", kv_pair.second}};
      to_return.push_back(value_map);
    }

    return to_return;
  }

  static void fromIntermediate(const YamlList& other,
                               SourceMap& value,
                               std::string& error) {
    value.clear();
    for (const auto& value_map : other) {
      if (!value_map.count("name")) {
        error = "invalid format! missing key 'name'";
        break;
      }

      if (!value_map.count("label")) {
        error = "invalid format! missing key 'label'";
        break;
      }

      value[std::stoi(value_map.at("label"))] = value_map.at("name");
    }
  }
};

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
  field(config.enable_lcd, "enable_lcd");
  field(config.timing_disabled, "timing_disabled");
  field(config.disable_timer_output, "disable_timer_output");
  field(config.enable_pgmo_logging, "enable_pgmo_logging");
  field(config.default_verbosity, "default_verbosity");
  field(config.default_num_threads, "default_num_threads");
  field(config.store_visualization_details, "store_visualization_details");
  config.map_window.setOptional();
  field(config.map_window, "map_window");
  field<LabelNameConversion>(config.label_names, "label_names");
  // the following subconfigs should not be namespaced
  field(config.frames, "frames", false);
  field(config.graph, "graph", false);
  field(config.label_space, "label_space", false);
}

GlobalInfo::GlobalInfo() : force_shutdown_(false) {}

void GlobalInfo::configureTimers() {
  ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
  timer.timing_disabled = config_.timing_disabled;
  timer.disable_output = config_.disable_timer_output;
}

void GlobalInfo::initFromConfig(const PipelineConfig& config, int robot_id) {
  config_ = config::checkValid(config);
  robot_prefix_ = RobotPrefixConfig(robot_id);

  configureTimers();

  if (!config_.label_space.label_remap_filepath.empty()) {
    label_remapper_ = LabelRemapper(config_.label_space.label_remap_filepath);
  }

  if (!config_.label_space.colormap_filepath.empty()) {
    label_colormap_ = SemanticColorMap::fromCsv(config_.label_space.colormap_filepath);
  }

  if (label_colormap_) {
    VLOG(2) << "Loaded label space colors:" << std::endl << *label_colormap_;
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

const std::map<uint32_t, std::string>& GlobalInfo::getLabelToNameMap() const {
  return config_.label_names;
}

const LabelSpaceConfig& GlobalInfo::getLabelSpaceConfig() const {
  return config_.label_space;
}

size_t GlobalInfo::getTotalLabels() const { return config_.label_space.total_labels; }

const LabelRemapper& GlobalInfo::getLabelRemapper() const { return label_remapper_; }

const SemanticColorMap* GlobalInfo::getSemanticColorMap() const {
  return label_colormap_.get();
}

SharedDsgInfo::Ptr GlobalInfo::createSharedDsg() const {
  auto graph_info = std::make_shared<SharedDsgInfo>(config_.graph);
  auto& graph = *graph_info->graph;

  const spark_dsg::Labelspace labelspace(getLabelToNameMap());
  if (labelspace) {
    labelspace.save(graph, "mesh");
    for (const auto& layer_name : config_.label_space.semantic_layers) {
      const auto key = graph.getLayerKey(layer_name);
      if (key) {
        labelspace.save(graph, key->layer, key->partition);
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
