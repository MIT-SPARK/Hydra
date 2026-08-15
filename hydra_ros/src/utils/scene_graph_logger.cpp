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
#include "hydra_ros/utils/scene_graph_logger.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/types/path.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <iomanip>

namespace hydra {

void declare_config(SceneGraphLogger::Config& config) {
  using namespace config;
  name("SceneGraphLogger::Config");
  field(config.output_every_num, "output_every_num");
  field<Path>(config.output_path, "output_path");
  checkCondition(!config.output_path.empty(), "output_path required");
}

SceneGraphLogger::SceneGraphLogger(const rclcpp::NodeOptions& options)
    : Node("scene_graph_logger", options),
      config(config::checkValid(config::fromContext<Config>())),
      curr_count_(0),
      curr_output_count_(0) {
  using namespace std::chrono_literals;

  bool exists = std::filesystem::exists(config.output_path);
  if (exists && !std::filesystem::is_directory(config.output_path)) {
    LOG(ERROR) << "Invalid output path: " << config.output_path;
    return;
  }

  if (!exists) {
    std::filesystem::create_directories(config.output_path);
  }

  timer_ = create_wall_timer(100ms, [this]() { spinOnce(); });
}

void SceneGraphLogger::spinOnce() {
  if (!receiver_) {
    ianvs::NodeHandle nh(*this, "");
    receiver_.reset(new DsgReceiver(nh));
    return;
  }

  if (!receiver_->updated()) {
    return;
  }

  ++curr_count_;
  receiver_->clearUpdated();
  if (config.output_every_num > 1 && curr_count_ % config.output_every_num != 1) {
    return;
  }

  std::stringstream ss;
  ss << "dsg_" << std::setfill('0') << std::setw(6) << curr_output_count_ << ".json";
  const auto path = config.output_path / ss.str();
  receiver_->graph()->save(path, false);
  ++curr_output_count_;
}

}  // namespace hydra

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(hydra::SceneGraphLogger)
