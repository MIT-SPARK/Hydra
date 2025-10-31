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

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <vector>

#include "hydra/common/label_remapper.h"
#include "hydra/common/label_space_config.h"
#include "hydra/common/robot_prefix_config.h"
#include "hydra/common/shared_dsg_info.h"
#include "hydra/input/sensor.h"

// TODO(nathan) bad....
#include "hydra/active_window/volumetric_window.h"

namespace hydra {

class SemanticColorMap;

struct FrameConfig {
  //! Body frame for robot constructing the scene graph (see REP 105)
  std::string robot = "base_link";
  //! Frame odometry estimates are relative to (see REP 105)
  std::string odom = "odom";
  //! Frame that the optimized scene graph is in (see REP 105)
  std::string map = "map";
};

void declare_config(FrameConfig& config);

struct PipelineConfig {
  //! If true, turn on internal loop closure detection
  bool enable_lcd = false;
  //! If true, disable all performance timers
  bool timing_disabled = false;
  //! If true, don't show latest elapsed for timers
  bool disable_timer_output = true;
  //! If true, forward pgmo custom logging to glog
  bool enable_pgmo_logging = true;
  //! If true, store additional details for the khronos spatio-temporal viualizer.
  bool store_visualization_details = false;
  //! Default settings for other modules. Can be overwritten by other module configs.
  int default_verbosity = 1;
  //! Default number of threads for multi-threaded integrators to use
  int default_num_threads = -1;  // -1 means use all available threads.
  //! Frame information for Hydra
  FrameConfig frames;
  //! Layer names for the scene graph that Hydra builds
  SharedDsgInfo::Config graph;
  //! Default windowing function that determines the active window
  config::VirtualConfig<VolumetricWindow> map_window{SpatialWindowChecker::Config()};
  //! Closed-set labelspace information
  LabelSpaceConfig label_space;
  //! Human readable category names for the labelspace
  std::map<uint32_t, std::string> label_names;
  //! @brief Layers in the scene graph that use the labelspace
  std::vector<std::string> semantic_layers{spark_dsg::DsgLayers::OBJECTS,
                                           spark_dsg::DsgLayers::MESH_PLACES};
};

void declare_config(PipelineConfig& config);

class GlobalInfo {
 public:
  static GlobalInfo& instance();

  static GlobalInfo& init(const PipelineConfig& config, int robot_id = 0);

  // this invalidates any instances (mostly intended for testing)
  static void reset();

  void setForceShutdown(bool force_shutdown);

  bool force_shutdown() const;

  const PipelineConfig& getConfig() const;

  const FrameConfig& getFrames() const;

  const RobotPrefixConfig& getRobotPrefix() const;

  const std::map<uint32_t, std::string>& getLabelToNameMap() const;

  const LabelSpaceConfig& getLabelSpaceConfig() const;

  size_t getTotalLabels() const;

  const LabelRemapper& getLabelRemapper() const;

  const SemanticColorMap* getSemanticColorMap() const;

  SharedDsgInfo::Ptr createSharedDsg() const;

  bool setSensor(const Sensor::Ptr& sensor, bool allow_override = true);

  Sensor::ConstPtr getSensor(const std::string& name) const;

  std::vector<std::string> getAvailableSensors() const;

  std::unique_ptr<VolumetricWindow> createVolumetricWindow() const;

 private:
  GlobalInfo();

  void configureTimers();

  void initFromConfig(const PipelineConfig& config, int robot_id);

 private:
  static std::unique_ptr<GlobalInfo> instance_;
  std::atomic<bool> force_shutdown_;

  PipelineConfig config_;
  RobotPrefixConfig robot_prefix_;
  LabelRemapper label_remapper_;
  std::shared_ptr<SemanticColorMap> label_colormap_;

  std::map<std::string, std::shared_ptr<const Sensor>> sensors_;
};

std::ostream& operator<<(std::ostream& out, const GlobalInfo& config);

}  // namespace hydra
