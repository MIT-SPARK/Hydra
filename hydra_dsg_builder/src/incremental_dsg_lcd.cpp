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
#include "hydra_dsg_builder/incremental_dsg_lcd.h"

#include <glog/logging.h>
#include <hydra_utils/timing_utilities.h>
#include <kimera_pgmo/utils/CommonFunctions.h>

namespace hydra {
namespace incremental {

using hydra::timing::ScopedTimer;
using lcd::LayerRegistrationConfig;

DsgLcd::DsgLcd(const RobotPrefixConfig& prefix,
               const DsgLcdModuleConfig& config,
               const SharedDsgInfo::Ptr& dsg,
               const SharedModuleState::Ptr& state)
    : prefix_(prefix),
      config_(config),
      dsg_(dsg),
      state_(state),
      lcd_graph_(new DynamicSceneGraph()) {
  lcd_detector_.reset(new lcd::DsgLcdDetector(config_.detector));
}

DsgLcd::~DsgLcd() { stop(); }

void DsgLcd::start() {
  spin_thread_.reset(new std::thread(&DsgLcd::spin, this));
  LOG(INFO) << "[DSG LCD] LCD started!";
}

void DsgLcd::stop() {
  VLOG(2) << "[DSG LCD] stopping lcd!";

  should_shutdown_ = true;
  if (spin_thread_) {
    VLOG(2) << "[DSG LCD] joining thread";
    spin_thread_->join();
    spin_thread_.reset();
    VLOG(2) << "[DSG LCD] joined thread";
  }
}

void DsgLcd::save(const std::string&) {}

void DsgLcd::spin() {
  if (!state_->lcd_queue) {
    LOG(ERROR) << "LCD queue required to run LCD";
    return;
  }

  while (!should_shutdown_) {
    bool has_data = state_->lcd_queue->poll();
    if (!has_data) {
      continue;
    }

    // TODO(nathan) consider config option for this
    // for now, we only update the lcd graph when the latest popped message
    // has a timestamp after the graph update (and only do lcd after an update)
    // this is okay as the frontend pushes output before marking the last update time
    // i.e.: lcd will always either be able to update the lcd graph copy or be able to
    // pop another message from the queue) This could potentially starve running LCD
    // when popping frontend outputs takes longer than it takes the frontend to
    // reproduce them (which is unlikely)
    spinOnceImpl(false);
  }
}

bool DsgLcd::spinOnce(bool force_update) {
  if (!state_->lcd_queue) {
    LOG(ERROR) << "LCD queue required to run LCD";
    return false;
  }

  bool has_data = state_->lcd_queue->poll();
  if (!has_data) {
    return false;
  }

  spinOnce(force_update);
  return true;
}

void DsgLcd::spinOnceImpl(bool force_update) {
  const size_t timestamp_ns = processFrontendOutput();

  {  // start critical section
    std::unique_lock<std::mutex> lock(dsg_->mutex);
    // TODO(nathan) add config option to force update anyways
    if (!force_update && timestamp_ns < dsg_->last_update_time) {
      return;
    }

    ScopedTimer spin_timer("lcd/merge_graph", timestamp_ns);
    lcd_graph_->mergeGraph(*dsg_->graph);
  }  // end critical section

  auto query_agent = getQueryAgentId(timestamp_ns);
  while (query_agent) {
    const Eigen::Vector3d query_pos = lcd_graph_->getPosition(*query_agent);
    const auto to_cache = getPlacesToCache(query_pos);

    if (!to_cache.empty()) {
      lcd_detector_->updateDescriptorCache(*lcd_graph_, to_cache, timestamp_ns);
    }

    auto time = lcd_graph_->getDynamicNode(*query_agent).value().get().timestamp;
    auto results = lcd_detector_->detect(*lcd_graph_, *query_agent, time.count());
    query_agent = getQueryAgentId(timestamp_ns);

    for (const auto& result : results) {
      // TODO(nathan) consider augmenting with gtsam key
      state_->backend_lcd_queue.push(result);
      LOG(WARNING) << "Found valid loop-closure: "
                   << NodeSymbol(result.from_node).getLabel() << " -> "
                   << NodeSymbol(result.to_node).getLabel();
    }
  }
}

size_t DsgLcd::processFrontendOutput() {
  const auto& msg = state_->lcd_queue->front();
  potential_lcd_root_nodes_.insert(potential_lcd_root_nodes_.end(),
                                   msg->archived_places.begin(),
                                   msg->archived_places.end());

  for (const auto& node : msg->new_agent_nodes) {
    agent_queue_.push(node);
  }

  size_t timestamp_ns = msg->timestamp_ns;
  state_->lcd_queue->pop();
  return timestamp_ns;
}

NodeIdSet DsgLcd::getPlacesToCache(const Eigen::Vector3d& agent_pos) {
  NodeIdSet to_cache;
  auto iter = potential_lcd_root_nodes_.begin();
  while (iter != potential_lcd_root_nodes_.end()) {
    const Eigen::Vector3d pos = lcd_graph_->getPosition(*iter);
    if ((agent_pos - pos).norm() < config_.descriptor_creation_horizon_m) {
      ++iter;
      continue;
    }

    to_cache.insert(*iter);
    iter = potential_lcd_root_nodes_.erase(iter);
  }

  return to_cache;
}

std::optional<NodeId> DsgLcd::getQueryAgentId(size_t stamp_ns) {
  if (agent_queue_.empty()) {
    return std::nullopt;
  }

  const auto& node = lcd_graph_->getDynamicNode(agent_queue_.top())->get();
  const auto prev_time = node.timestamp;

  if (!node.hasParent()) {
    LOG(ERROR) << "Found agent node without parent: "
               << NodeSymbol(agent_queue_.top()).getLabel() << ". Discarding!";
    agent_queue_.pop();
    return std::nullopt;
  }

  std::chrono::duration<double> diff_s = std::chrono::nanoseconds(stamp_ns) - prev_time;
  // we consider should_shutdown_ here to make sure we're not waiting on popping from
  // the LCD queue while not getting new place messages
  if (!should_shutdown_ && diff_s.count() < config_.lcd_agent_horizon_s) {
    return std::nullopt;
  }

  if (should_shutdown_ && (diff_s.count() < config_.lcd_agent_horizon_s)) {
    LOG(ERROR) << "Forcing pop of node " << NodeSymbol(agent_queue_.top()).getLabel()
               << " from lcd queue due to shutdown: "
               << ", diff: " << diff_s.count() << " / " << config_.lcd_agent_horizon_s;
  }

  auto valid_node = agent_queue_.top();
  agent_queue_.pop();
  return valid_node;
}

}  // namespace incremental
}  // namespace hydra
