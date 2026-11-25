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
#include <hydra/utils/logging.h>
#include <spark_dsg/labelspace.h>

#include "hydra/active_window/reconstruction_module.h"
#include "hydra/active_window/volumetric_window.h"
#include "hydra/backend/backend_module.h"
#include "hydra/common/run_context.h"
#include "hydra/common/run_settings.h"
#include "hydra/frontend/graph_builder.h"
#include "hydra/loop_closure/loop_closure_module.h"  // IWYU pragma: keep
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

SharedDsgInfo::Ptr buildGraph(const RunContext& context) {
  auto info = std::make_shared<SharedDsgInfo>(context.config.graph);
  auto& graph = *info->graph;

  const spark_dsg::Labelspace labelspace(context.labelspace().label_names);
  if (labelspace) {
    labelspace.save(graph, "mesh", false);
    for (const auto& layer_name : context.config.semantic_layers) {
      if (graph.layer_names().count(layer_name)) {
        labelspace.save(graph, layer_name);
      } else {
        HYDRA_LOG(WARNING, "Skipping saving labelspace for unknown '{}'", layer_name);
      }
    }
  }

  return info;
}

}  // namespace

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
    : VerbosityConfig("pipeline"),
      map_window(SpatialWindowChecker::Config()),
      active_window(ReconstructionModule::Config()),
      frontend(GraphBuilder::Config()),
      backend(BackendModule::Config()) {}

HydraPipeline::HydraPipeline(const Config& config, RunContext::ConstPtr context)
    : config(config), context_(std::move(context)) {
  context_->activate();

  initState();
}

HydraPipeline::~HydraPipeline() { stop(); }

void HydraPipeline::init() {
  frontend_ = config.frontend.create(
      context_, frontend_dsg_, state_, config.map_window.create());

  ActiveWindowModule::OutputQueue::Ptr queue;
  if (frontend_) {
    queue = frontend_->queue();
  }

  active_window_ =
      config.active_window.create(context_, queue, config.map_window.create());

  backend_ = config.backend.create(context_, backend_dsg_, state_);
  if (frontend_ && backend_) {
    frontend_->setBackendQueue(backend_->queue());
  }

  LoopClosureModule::OutputQueue::Ptr lcd_queue;
  if (backend_) {
    lcd_queue = backend_->lcd_queue();
  }

  lcd_ = config.lcd.create(context_, state_, lcd_queue);
  if (frontend_ && lcd_) {
    frontend_->setLcdQueue(lcd_->queue());
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

void HydraPipeline::stop(bool force) {
  if (active_window_) {
    active_window_->stop(force);
  }

  if (frontend_) {
    frontend_->stop(force);
  }

  if (backend_) {
    backend_->stop(force);
  }

  if (lcd_) {
    lcd_->stop(force);
  }

  logging::logger()->flush();
}

void HydraPipeline::save(const DataDirectory* output) const {
  const auto& logs = output ? *output : context_->logs;
  if (!logs) {
    return;
  }

  const auto log_dir = logs.path();

  auto node = config::toYaml(context_->config);
  const auto pipeline_node = config::toYaml(config);
  for (const auto& entry : pipeline_node) {
    node[entry.first.as<std::string>()] = entry.second;
  }

  node["labelspace"] = config::toYaml(context_->labelspace());
  node["labelspace"]["type"] = "from_config";
  std::ofstream fout(log_dir / "hydra_config.yaml");
  fout << node;

  active_window_->save(&logs);
  frontend_->save(&logs);
  backend_->save(&logs);
  if (lcd_) {
    lcd_->save(&logs);
  }

  // save timing information to avoid destructor weirdness with singletons
  MLOG(0, "Saving timing information to {}", log_dir);
  const auto& timer = ElapsedTimeRecorder::instance();
  timer.logTimers(logs.path("timing"));
  timer.logStats(log_dir / "timing_stats.csv");
  MLOG(0, "Saved timing information");
  logging::logger()->flush();
}

void HydraPipeline::showModules() const {
  MLOG(1, "Active Window:\n{}", active_window_->show());
  MLOG(1, "Frontend:\n{}", frontend_->show());
  MLOG(1, "Backend:\n{}", backend_->show());
  if (lcd_) {
    MLOG(1, "Loop Closure Detection:\n{}", lcd_->show());
  }
}

void HydraPipeline::initState() {
  ElapsedTimeRecorder::reset();
  auto& timer = ElapsedTimeRecorder::instance();
  timer.timing_disabled = RunSettings::instance().timing_disabled;
  timer.disable_output = RunSettings::instance().disable_timer_output;

  frontend_dsg_ = buildGraph(*context_);
  backend_dsg_ = buildGraph(*context_);
  state_.reset(new SharedModuleState());
  state_->lcd_graph = buildGraph(*context_);
  state_->backend_graph = buildGraph(*context_);
}

}  // namespace hydra
