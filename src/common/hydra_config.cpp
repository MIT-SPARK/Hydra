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
#include "hydra/common/hydra_config.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>

#include "hydra/common/common.h"
#include "hydra/common/semantic_color_map.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using ColorMapPtr = std::shared_ptr<SemanticColorMap>;
using timing::ElapsedTimeRecorder;

decltype(HydraConfig::instance_) HydraConfig::instance_;

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

void declare_config(PipelineConfig& conf) {
  using namespace config;
  name("PipelineConfig");
  field(conf.enable_reconstruction, "enable_reconstruction");
  field(conf.enable_lcd, "enable_lcd");
  field(conf.map, "reconstruction/map");
  field<LabelNameConversion>(conf.label_names, "label_names");
  field(conf.room_colors, "room_colors");
  // the following subconfigs should not be namespaced
  field(conf.logs, "logs", false);
  field(conf.frames, "frames", false);
  field(conf.label_space, "label_space", false);
}

void saveTimingInformation(const LogSetup& log_config) {
  if (!log_config.valid()) {
    return;
  }

  LOG(INFO) << "[Hydra] saving timing information to " << log_config.getLogDir();
  const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
  timer.logAllElapsed(log_config);
  timer.logStats(log_config.getTimerFilepath());
  LOG(INFO) << "[Hydra] saved timing information";
}

HydraConfig::HydraConfig() : force_shutdown_(false) {
  label_colormap_.reset(new SemanticColorMap());
}

void HydraConfig::configureTimers() {
  ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
  timer.timing_disabled = config_.timing_disabled;
  timer.disable_output = config_.disable_timer_output;
  if (timer.timing_disabled) {
    return;
  }

  if (!logs_ || !logs_->valid()) {
    return;
  }

  if (logs_->config().log_timing_incrementally) {
    timer.setupIncrementalLogging(logs_);
  }
}

void HydraConfig::checkFrozen() const {
  if (!frozen_) {
    LOG(ERROR) << "HydraConfig is not frozen! Call init with freeze set to 'true' "
                  "before using config";
    throw std::runtime_error("config not frozen");
  }
}

void HydraConfig::initFromConfig(const PipelineConfig& config, int robot_id) {
  config_ = config::checkValid(config);
  robot_prefix_ = RobotPrefixConfig(robot_id);
  logs_ = std::make_shared<LogSetup>(config_.logs);
  configureTimers();

  if (!config_.label_space.colormap.empty()) {
    SemanticColorMap::ColorToLabelMap new_colors;
    for (auto&& [id, color] : config_.label_space.colormap) {
      new_colors[voxblox::Color(color[0], color[1], color[2], 255)] = id;
    }

    label_colormap_.reset(new SemanticColorMap(new_colors));
  } else if (!config_.label_space.colormap_filepath.empty()) {
    label_colormap_ = SemanticColorMap::fromCsv(config_.label_space.colormap_filepath);
  } else {
    label_colormap_ = SemanticColorMap::randomColors(config_.label_space.total_labels);
  }

  if (label_colormap_) {
    VLOG(VLEVEL_TRACE) << "Loaded label space colors:" << std::endl << *label_colormap_;
  }
}

HydraConfig& HydraConfig::instance() {
  if (!instance_) {
    instance_.reset(new HydraConfig());
  }
  return *instance_;
}

HydraConfig& HydraConfig::init(const PipelineConfig& config,
                               int robot_id,
                               bool freeze) {
  auto& curr = instance();
  if (curr.frozen_) {
    LOG(ERROR) << "Failed to initialize HydraConfig as config was already frozen";
    throw std::runtime_error("hydra global config is frozen");
  }

  curr.initFromConfig(config, robot_id);
  // TODO(nathan) print?
  curr.frozen_ = freeze;
  return curr;
}

void HydraConfig::reset() { instance_.reset(new HydraConfig()); }

void HydraConfig::exit() {
  auto& curr = instance();

  // save timing information to avoid destructor weirdness with singletons
  if (curr.logs_) {
    saveTimingInformation(*curr.logs_);
    curr.logs_.reset();
  }

  // TODO(nathan) see if anything else needs to be saved;
}

void HydraConfig::setForceShutdown(bool force_shutdown) {
  force_shutdown_ = force_shutdown;
}

ColorMapPtr HydraConfig::setRandomColormap() {
  label_colormap_ = SemanticColorMap::randomColors(config_.label_space.total_labels);
  return label_colormap_;
}

bool HydraConfig::force_shutdown() const { return force_shutdown_; }

const PipelineConfig& HydraConfig::getConfig() const {
  checkFrozen();
  return config_;
}

const FrameConfig& HydraConfig::getFrames() const {
  checkFrozen();
  return config_.frames;
}

const RobotPrefixConfig& HydraConfig::getRobotPrefix() const {
  checkFrozen();
  return robot_prefix_;
}

const LogSetup::Ptr& HydraConfig::getLogs() const {
  checkFrozen();
  return logs_;
}

const VolumetricMap::Config& HydraConfig::getMapConfig() const {
  checkFrozen();
  return config_.map;
}

const ColorArray& HydraConfig::getRoomColor(size_t index) const {
  checkFrozen();
  return config_.room_colors.at(index % config_.room_colors.size());
}

const std::map<uint32_t, std::string>& HydraConfig::getLabelToNameMap() const {
  checkFrozen();
  return config_.label_names;
}

const LabelSpaceConfig& HydraConfig::getLabelSpaceConfig() const {
  checkFrozen();
  return config_.label_space;
}

size_t HydraConfig::getTotalLabels() const {
  checkFrozen();
  return config_.label_space.total_labels;
}

SharedDsgInfo::Ptr HydraConfig::createSharedDsg() const {
  checkFrozen();
  return std::make_shared<SharedDsgInfo>(config_.layer_id_map, config_.mesh_layer_id);
}

ColorMapPtr HydraConfig::getSemanticColorMap() const { return label_colormap_; }

std::ostream& operator<<(std::ostream& out, const HydraConfig& config) {
  out << config::toString(config.getConfig());
  return out;
}

}  // namespace hydra
