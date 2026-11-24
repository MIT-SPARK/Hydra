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
#include "hydra/common/run_context.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/labelspace.h>
#include <spark_dsg/scene_graph.h>

#include "hydra/common/semantic_color_map.h"
#include "hydra/utils/pgmo_glog_sink.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {

using ColorMapPtr = std::shared_ptr<SemanticColorMap>;
using timing::ElapsedTimeRecorder;

void declare_config(RunContext::MeshFields& config) {
  using namespace config;
  name("RunContext::Config::MeshFields");
  field(config.with_colors, "with_colors");
  field(config.with_first_seen_stamps, "with_first_seen_stamps");
  field(config.with_labels, "with_labels");
}

void declare_config(RunContext::Frames& frames) {
  using namespace config;
  name("RunContext::Config::Frames");
  field(frames.robot, "robot_frame");
  field(frames.odom, "odom_frame");
  field(frames.map, "map_frame");
}

void declare_config(RunContext& config) {
  using namespace config;
  name("RunContext::Config");
  field(config.timing_disabled, "timing_disabled");
  field(config.disable_timer_output, "disable_timer_output");
  field(config.enable_pgmo_logging, "enable_pgmo_logging");
  field(config.default_verbosity, "default_verbosity");
  field(config.default_num_threads, "default_num_threads");
  field(config.store_visualization_details, "store_visualization_details");
  field(config.mesh, "mesh");
  field(config.semantic_layers, "semantic_layers");
  field(config.graph, "graph");

  // the following subconfigs should not be namespaced
  field(config.frames, "frames", false);
}

RunContext::RunContext(size_t robot_id) : robot_id_(robot_id), force_shutdown_(false) {}

void RunContext::initFromConfig(const PipelineConfig& config, int robot_id) {
  config_ = config::checkValid(config);
  robot_prefix_ = RobotPrefixConfig(robot_id);

  auto& timer = ElapsedTimeRecorder::instance();
  timer.timing_disabled = config_.timing_disabled;
  timer.disable_output = config_.disable_timer_output;

  if (config_.enable_pgmo_logging) {
    logging::Logger::addSink("glog", std::make_shared<PgmoGlogSink>());
  }
}

void RunContext::setForceShutdown(bool force_shutdown) {
  force_shutdown_ = force_shutdown;
}

bool RunContext::force_shutdown() const { return force_shutdown_; }

RobotPrefixConfig RunContext::robot_prefix() const {
  return RobotPrefixConfig(robot_id_);
}

spark_dsg::Mesh::Ptr RunContext::createMesh() const {
  // we force the mesh to have last seen stamps
  return std::make_shared<spark_dsg::Mesh>(
      mesh.with_colors, true, mesh.with_labels, mesh.with_first_seen_stamps);
}

}  // namespace hydra
