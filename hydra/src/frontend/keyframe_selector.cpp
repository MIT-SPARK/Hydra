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
#include "hydra/frontend/keyframe_selector.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include "hydra/active_window/volumetric_window.h"
#include "hydra/common/global_info.h"
#include "hydra/odometry/pose_graph_from_odom.h"
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilderFunctor,
                                   KeyframeSelector,
                                   KeyframeSelector::Config>("KeyframeSelector");

}

using hydra::timing::ScopedTimer;

void declare_config(KeyframeSelector::Config& config) {
  using namespace config;
  name("KeyframeSelector::Config");
  base<VerbosityConfig>(config);
  config.pose_graph_tracker.setOptional();
  field(config.pose_graph_tracker, "pose_graph_tracker");
  field(config.view_database, "view_database");
}

KeyframeSelector::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("keyframes")),
      pose_graph_tracker(PoseGraphFromOdom::Config()) {}

KeyframeSelector::KeyframeSelector(const Config& config)
    : config(config::checkValid(config)),
      tracker_(config.pose_graph_tracker.create()),
      view_database_(config.view_database) {}

void KeyframeSelector::call(const ActiveWindowOutput& input,
                            SharedDsgInfo& dsg,
                            FrontendOutput& output,
                            const VolumetricWindow* window) {
  if (!tracker_) {
    LOG_FIRST_N(WARNING, 1) << "pose graph tracking disabled";
    return;
  }

  ScopedTimer timer("frontend/update_posegraph", input.timestamp_ns);

  PoseGraphPacket packet;
  for (const auto& data : input.sensor_data) {
    const auto curr_packet = tracker_->update(data->timestamp_ns, data->world_T_body);
    packet.updateFrom(curr_packet);
  }

  const auto& prefix = GlobalInfo::instance().getRobotPrefix();

  // TODO(nathan) thinking about locking more
  std::lock_guard<std::mutex> lock(dsg.mutex);
  const auto new_node_ids = packet.addToGraph(*dsg.graph, prefix.id);

  output.agent_updates = packet;
  output.new_agent_nodes = new_node_ids;
}

void KeyframeSelector::callPostUpdate(SharedDsgInfo& dsg, FrontendOutput& output) {
  view_database_.updateAssignments(
      *dsg.graph,
      [&](const Eigen::Vector3d& pos, uint64_t timestamp) { return false; });
}

/*
        if (!map_window_) {
          return false;
        }

        const auto fmt = getDefaultFormat(3);
        MLOG(2) << "view @ " << timestamp << "[ns]: " << pos.format(fmt) << " vs. "
                << msg->world_T_body().translation().format(fmt);
        return !map_window_->inBounds(
            msg->timestamp_ns, msg->world_T_body(), timestamp, pos);
            */

}  // namespace hydra
