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
#include "hydra/common/hydra_pipeline.h"

#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>

#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ElapsedTimeRecorder;

HydraPipeline::HydraPipeline(const PipelineConfig& pipeline_config,
                             int robot_id,
                             int config_verbosity)
    : config_verbosity_(config_verbosity) {
  const auto& config = GlobalInfo::init(pipeline_config, robot_id);
  frontend_dsg_ = config.createSharedDsg();
  backend_dsg_ = config.createSharedDsg();
  shared_state_.reset(new SharedModuleState());
  shared_state_->lcd_graph = config.createSharedDsg();
  shared_state_->backend_graph = config.createSharedDsg();
  LOG(INFO) << "[Hydra] Initialized pipeline with:\n" << config;
}

void HydraPipeline::init() {}

HydraPipeline::~HydraPipeline() {}

std::string makeBanner(const std::string& message,
                       size_t print_width,
                       char fill,
                       bool with_header = true,
                       bool with_footer = false) {
  std::stringstream ss;
  if (with_header) {
    ss << std::string(print_width, fill) << std::endl;
  }
  const auto msg_size = message.size() + 3;
  const auto spacing = msg_size >= print_width ? 0 : print_width - msg_size;
  ss << fill << " " << message << (spacing ? std::string(spacing, ' ') + fill : "")
     << std::endl;
  if (with_footer) {
    ss << std::string(print_width, fill) << std::endl;
  }
  return ss.str();
}

std::string HydraPipeline::getModuleInfo(const std::string& name,
                                         const Module* mod) const {
  const auto print_width = config::Settings().printing.width;
  std::stringstream ss;
  ss << makeBanner(name, print_width, '*', true, true);
  if (!mod) {
    ss << "UNITIALIZED MODULE!" << std::endl;
  } else {
    const auto info = mod->printInfo();
    if (!info.empty()) {
      ss << info << std::endl;
    }
  }
  ss << std::string(print_width, '*') << std::endl;
  return ss.str();
}

void HydraPipeline::showModules() const {
  const auto print_width = config::Settings().printing.width;
  std::stringstream ss;
  ss << std::endl << makeBanner("Modules", print_width, '=', true, true);
  for (auto&& [name, mod] : modules_) {
    ss << std::endl << getModuleInfo(name, mod.get());
  }
  VLOG(config_verbosity_) << ss.str();
}

void HydraPipeline::start() {
  VLOG(config_verbosity_) << std::endl << getModuleInfo("input", input_module_.get());
  showModules();

  if (input_module_) {
    input_module_->start();
  }

  for (auto&& [name, mod] : modules_) {
    if (!mod) {
      LOG(FATAL) << "Found unitialized module: " << name;
      continue;
    }

    mod->start();
  }

  if (input_module_) {
    modules_["input"] = input_module_;
  }
}

void HydraPipeline::stop() {
  for (auto&& [name, mod] : modules_) {
    if (!mod) {
      LOG(FATAL) << "Found unitialized module: " << name;
      continue;
    }

    mod->stop();
  }
}

void HydraPipeline::save(const DataDirectory& logs) const {
  if (!logs) {
    return;
  }

  auto& info = GlobalInfo::instance();
  auto node = config::toYaml(info.getConfig());
  for (const auto& name : info.getAvailableSensors()) {
    const auto sensor = info.getSensor(name);
    if (sensor) {
      node["sensors"][name] = info.getSensor(name)->dump();
    }
  }

  std::filesystem::path log_dir = logs.path();
  std::ofstream fout(log_dir / "hydra_config.yaml");
  fout << node;

  for (auto&& [name, mod] : modules_) {
    if (!mod) {
      LOG(FATAL) << "Found unitialized module: " << name;
      continue;
    }

    mod->save(logs);
  }

  // save timing information to avoid destructor weirdness with singletons
  LOG(INFO) << "[Hydra] saving timing information to " << log_dir;
  const ElapsedTimeRecorder& timer = ElapsedTimeRecorder::instance();
  timer.logTimers(logs.path("timing"));
  timer.logStats(log_dir / "timing_stats.csv");
  LOG(INFO) << "[Hydra] saved timing information";
}

}  // namespace hydra
