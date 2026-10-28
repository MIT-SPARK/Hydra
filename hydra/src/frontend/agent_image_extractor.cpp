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
#include "hydra/frontend/agent_image_extractor.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/node_symbol.h>

#include "hydra/common/global_info.h"
#include "hydra/frontend/subkeyframe_anchor.h"
#include "hydra/input/camera.h"
#include "hydra/utils/timing_utilities.h"

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilderFunctor,
                                   AgentImageExtractor,
                                   AgentImageExtractor::Config>("AgentImageExtractor");

}  // namespace

using spark_dsg::AgentNodeAttributes;
using spark_dsg::NodeSymbol;
using spark_dsg::SceneGraph;
using timing::ScopedTimer;

void declare_config(AgentImageExtractor::Config& config) {
  using namespace config;
  name("AgentImageExtractor::Config");
  base<VerbosityConfig>(config);
  field(config.sensor_name, "sensor_name");
  field(config.gate, "gate");
  field(config.max_pairing_time_diff_s, "max_pairing_time_diff_s", "s");
  field(config.max_buffered_frames, "max_buffered_frames");
  field(config.max_deferred_updates, "max_deferred_updates");

  check(config.max_pairing_time_diff_s, GE, 0.0, "max_pairing_time_diff_s");
  check(config.max_buffered_frames, GT, size_t(0), "max_buffered_frames");
  check(config.max_deferred_updates, GT, size_t(0), "max_deferred_updates");
}

AgentImageExtractor::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("agent_images")) {}

AgentImageExtractor::AgentImageExtractor(const Config& config)
    : config(config::checkValid(config)),
      gate_(config.gate),
      store_(GlobalInfo::instance().imageStore()) {
  LOG_IF(INFO, !store_) << config.prefix
                        << "No image store (image_store.path); agent keyframes are "
                           "not saved";
}

void AgentImageExtractor::call(const ActiveWindowOutput& msg,
                               SharedDsgInfo&,
                               FrontendOutput&,
                               const VolumetricWindow*) {
  if (!store_) {
    return;  // disabled: nothing is buffered
  }

  // every input (including inputs collated into one update) produces an agent node
  ScopedTimer timer("frontend/agent_images_buffer", msg.timestamp_ns, true, 1, false);
  for (const auto& data : msg.sensor_data) {
    if (!data) {
      continue;
    }

    const auto& sensor = data->getSensor();
    if (!config.sensor_name.empty() && sensor.name != config.sensor_name) {
      continue;
    }

    // the input shares its pixel buffers with the active window, so the images are
    // copied (into their on-disk format)
    BufferedFrame frame;
    frame.timestamp_ns = data->timestamp_ns;
    frame.sensor = sensor.name;
    frame.world_T_body = data->world_T_body;
    if (!data->color_image.empty()) {
      frame.color = colorToKeyframe(data->color_image);
    }

    frame.depth = depthToKeyframe(data->depth_image);

    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!calibs_.count(sensor.name)) {
      const auto camera = dynamic_cast<const Camera*>(&sensor);
      if (camera) {
        calibs_[sensor.name] = CameraCalib::fromCamera(*camera);
      } else {
        LOG_FIRST_N(WARNING, 1) << config.prefix << "Sensor '" << sensor.name
                                << "' is not a camera; storing empty calibration";
        calibs_[sensor.name] = CameraCalib();
      }
    }

    if (!frames_.empty() && frames_.back().timestamp_ns >= frame.timestamp_ns) {
      continue;  // duplicate or out-of-order input: frames must stay sorted
    }

    frames_.push_back(std::move(frame));
    while (frames_.size() > config.max_buffered_frames) {
      frames_.pop_front();
    }
  }
}

std::optional<size_t> AgentImageExtractor::findFrame(
    const std::vector<BufferedFrame>& frames,
    size_t search_start,
    uint64_t timestamp_ns) const {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::nanoseconds;
  const auto tolerance_ns = static_cast<uint64_t>(
      duration_cast<nanoseconds>(duration<double>(config.max_pairing_time_diff_s))
          .count());

  std::optional<size_t> best;
  uint64_t best_diff = 0;
  for (size_t i = search_start; i < frames.size(); ++i) {
    const auto frame_ns = frames[i].timestamp_ns;
    const uint64_t diff =
        frame_ns > timestamp_ns ? frame_ns - timestamp_ns : timestamp_ns - frame_ns;
    if (diff > tolerance_ns) {
      if (frame_ns > timestamp_ns) {
        break;  // frames are sorted, so later frames are further away
      }

      continue;
    }

    if (!best || diff < best_diff) {
      best = i;
      best_diff = diff;
    }
  }

  return best;
}

std::vector<AgentImageExtractor::Keyframe> AgentImageExtractor::selectKeyframes(
    const SceneGraph& graph, const std::vector<BufferedFrame>& frames) {
  std::vector<Keyframe> keyframes;
  const uint64_t newest_frame_ns = frames.empty() ? 0 : frames.back().timestamp_ns;

  // each frame is paired with at most one node and nodes are visited in time order
  size_t search_start = 0;
  while (!pending_nodes_.empty()) {
    const auto node_id = pending_nodes_.front();
    const auto node = graph.findNode(node_id);
    const auto attrs = node ? node->tryAttributes<AgentNodeAttributes>() : nullptr;
    if (!attrs) {
      pending_nodes_.pop_front();
      continue;
    }

    // pair the node with the frame it was created from instead of the latest frame
    const uint64_t node_ns = attrs->timestamp.count();
    const auto frame_idx = findFrame(frames, search_start, node_ns);
    if (!frame_idx && node_ns > newest_frame_ns) {
      // the frame may still arrive, but only wait a bounded number of updates
      if (++deferred_count_ <= config.max_deferred_updates) {
        break;  // later nodes cannot be paired either
      }

      LOG_EVERY_N(WARNING, 10)
          << config.prefix << "Skipping agent " << NodeSymbol(node_id).str() << " @ "
          << node_ns << " [ns]: no sensor frame after " << deferred_count_
          << " updates";
    }

    pending_nodes_.pop_front();
    deferred_count_ = 0;
    last_decided_ns_ = node_ns;
    if (!frame_idx) {
      MLOG(2) << "No sensor frame within " << config.max_pairing_time_diff_s
              << " [s] of agent " << NodeSymbol(node_id).str() << " @ " << node_ns
              << " [ns]";
      continue;
    }

    search_start = *frame_idx + 1;
    if (gate_.shouldTrigger(attrs->position, attrs->world_R_body)) {
      Eigen::Isometry3d world_T_node = Eigen::Isometry3d::Identity();
      world_T_node.translation() = attrs->position;
      world_T_node.linear() = attrs->world_R_body.toRotationMatrix();
      keyframes.push_back({node_id, node_ns, world_T_node, frames[*frame_idx]});
    }
  }

  return keyframes;
}

void AgentImageExtractor::callPostUpdate(SharedDsgInfo& dsg, FrontendOutput& output) {
  if (!store_) {
    return;
  }

  ScopedTimer timer("frontend/agent_images", output.timestamp_ns, true, 1, false);
  pending_nodes_.insert(pending_nodes_.end(),
                        output.new_agent_nodes.begin(),
                        output.new_agent_nodes.end());
  if (pending_nodes_.empty()) {
    return;
  }

  // copies of the frame headers keep the store out of the critical sections
  std::vector<BufferedFrame> frames;
  std::map<std::string, CameraCalib> calibs;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frames.assign(frames_.begin(), frames_.end());
    calibs = calibs_;
  }

  std::vector<Keyframe> keyframes;
  {
    std::lock_guard<std::mutex> lock(dsg.mutex);
    keyframes = selectKeyframes(*dsg.graph, frames);
  }

  // frames older than the last decided node can never be paired
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    while (!frames_.empty() && frames_.front().timestamp_ns <= last_decided_ns_) {
      frames_.pop_front();
    }
  }

  if (keyframes.empty()) {
    return;
  }

  const auto robot_id = GlobalInfo::instance().getRobotPrefix().id;
  for (const auto& keyframe : keyframes) {
    const auto& frame = keyframe.frame;
    ImageStore::Frame record;
    record.robot_id = robot_id;
    const auto calib = calibs.find(frame.sensor);
    record.sensor_id = store_->addSensor(
        robot_id, frame.sensor, calib == calibs.end() ? CameraCalib() : calib->second);
    record.timestamp_ns = frame.timestamp_ns;
    record.kind = ImageStore::FrameKind::Keyframe;
    record.world_T_body = frame.world_T_body;
    record.anchor_node = keyframe.node;
    record.anchor_T_body =
        computeRelativeTransform(keyframe.world_T_node, frame.world_T_body);
    store_->addFrame(record, frame.color, frame.depth);
    MLOG(3) << "Saved keyframe for agent " << NodeSymbol(keyframe.node).str() << " @ "
            << keyframe.node_ns << " [ns]";
  }
}

}  // namespace hydra
