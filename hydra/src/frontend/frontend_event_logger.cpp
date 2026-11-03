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
#include "hydra/frontend/frontend_event_logger.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <spark_dsg/node_attributes.h>

#include <sstream>

#include "hydra/common/global_info.h"
#include "hydra/utils/data_directory.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilder::Sink,
                                   FrontendEventLogger,
                                   FrontendEventLogger::Config>("FrontendEventLogger");

}  // namespace

void declare_config(FrontendEventLogger::Config& config) {
  using namespace config;
  name("FrontendEventLogger::Config");
  field<Path>(config.path, "path");
  field(config.log_agents, "log_agents");
}

FrontendEventLogger::FrontendEventLogger(const Config& config)
    : config(config::checkValid(config)) {
  if (!config.path.empty()) {
    const auto robot_id = GlobalInfo::instance().getRobotPrefix().id;
    log_ = std::make_unique<EventLog>(
        config.path,
        "\"source\": \"frontend\", \"robot_id\": " + std::to_string(robot_id));
  }
}

void FrontendEventLogger::call(uint64_t timestamp_ns,
                               const spark_dsg::SceneGraph& graph,
                               const FrontendOutput& output) const {
  if (!log_) {
    return;
  }

  std::vector<std::string> lines;
  for (const auto& event : output.binding_events) {
    const auto& binding = event.binding;
    std::stringstream ss;
    ss << "{\"ev\": \"" << (binding.end ? "end" : "bind")
       << "\", \"stamp_ns\": " << event.timestamp_ns
       << ", \"layer\": " << jsonString(event.layer)
       << ", \"track_id\": " << binding.track_id
       << ", \"node_id\": " << binding.node_id;
    if (binding.end) {
      ss << ", \"reason\": " << jsonString(*binding.end);
    }
    ss << "}";
    lines.push_back(ss.str());
  }

  if (config.log_agents) {
    for (const auto node_id : output.new_agent_nodes) {
      const auto node = graph.findNode(node_id);
      if (!node) {
        continue;
      }

      const auto attrs = node->tryAttributes<spark_dsg::AgentNodeAttributes>();
      if (!attrs) {
        continue;
      }

      const Eigen::Matrix3d R = attrs->world_R_body.toRotationMatrix();
      std::stringstream ss;
      ss << "{\"ev\": \"agent\", \"stamp_ns\": " << timestamp_ns
         << ", \"node_id\": " << node_id
         << ", \"timestamp_ns\": " << attrs->timestamp.count()
         << ", \"world_T_body\": [";
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          const double value =
              r < 3 ? (c < 3 ? R(r, c) : attrs->position(r)) : (c < 3 ? 0.0 : 1.0);
          ss << (r + c ? ", " : "") << jsonNumber(value);
        }
      }
      ss << "]}";
      lines.push_back(ss.str());
    }
  }

  log_->append(lines);
}

std::string FrontendEventLogger::printInfo() const { return config::toString(config); }

}  // namespace hydra
