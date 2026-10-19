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

#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/settings.h>
#include <glog/logging.h>

#include "hydra/active_window/reconstruction_module.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/backend/backend_module.h"
#include "hydra/common/global_info.h"
#include "hydra/frontend/graph_builder.h"
#include "hydra/loop_closure/loop_closure_module.h"  // IWYU pragma: keep
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using hydra::timing::ElapsedTimeRecorder;

void declare_config(HydraPipeline::Config& config) {
  using namespace config;
  name("HydraPipeline::Config");
  base<VerbosityConfig>(config);
  config.map_window.setOptional();
  field(config.map_window, "map_window");
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
  config.lcd.setOptional();
  field(config.lcd, "lcd");
}

HydraPipeline::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("pipeline")),
      map_window(SpatialWindowChecker::Config()),
      active_window(ReconstructionModule::Config()),
      frontend(GraphBuilder::Config()),
      backend(BackendModule::Config()) {}

HydraPipeline::HydraPipeline(const Config& config) : config(config) {
  auto& info = GlobalInfo::instance();
  frontend_dsg_ = info.createSharedDsg();
  backend_dsg_ = info.createSharedDsg();
  state_.reset(new SharedModuleState());
  state_->lcd_graph = info.createSharedDsg();
  state_->backend_graph = info.createSharedDsg();
}

void HydraPipeline::init() {
  frontend_ = config.frontend.create(frontend_dsg_, state_, config.map_window.create());
  backend_ = config.backend.create(backend_dsg_, state_);
  const auto queue = frontend_->queue();
  active_window_ = config.active_window.create(queue, config.map_window.create());
  lcd_ = config.lcd.create(state_);
  if (lcd_) {
    frontend_->setLcdQueue(lcd_->queue());
  }
}

HydraPipeline::~HydraPipeline() {}

void HydraPipeline::showModules() const {
  MLOG(1) << "Active Window:\n" << active_window_->printInfo();
  MLOG(1) << "Frontend:\n" << frontend_->printInfo();
  MLOG(1) << "Backend:\n" << backend_->printInfo();
  if (lcd_) {
    MLOG(1) << "Loop Closure Detection:\n" << lcd_->printInfo();
  }
}

void HydraPipeline::start() {
  showModules();

  active_window_->start();
  frontend_->start();
  backend_->start();
  if (lcd_) {
    lcd_->start();
  }
}

void HydraPipeline::stop() {
  active_window_->stop();
  frontend_->stop();
  backend_->stop();
  if (lcd_) {
    lcd_->stop();
  }
}

void HydraPipeline::save(const DataDirectory& logs) const {
  if (!logs) {
    return;
  }

  const auto log_dir = logs.path();

  auto& info = GlobalInfo::instance();
  auto node = config::toYaml(info.getConfig());
  std::ofstream fout(log_dir / "hydra_config.yaml");
  fout << node;

  active_window_->save(logs);
  frontend_->save(logs);
  backend_->save(logs);
  if (lcd_) {
    lcd_->save(logs);
  }

  // save timing information to avoid destructor weirdness with singletons
  MLOG(0) << "Saving timing information to " << log_dir;
  const auto& timer = ElapsedTimeRecorder::instance();
  timer.logTimers(logs.path("timing"));
  timer.logStats(log_dir / "timing_stats.csv");
  MLOG(0) << "Saved timing information";
}

}  // namespace hydra
