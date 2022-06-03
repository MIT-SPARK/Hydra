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

DsgLcd::DsgLcd(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg)
    : nh_(nh), dsg_(dsg), lcd_graph_(new DynamicSceneGraph()) {
  // TODO(nathan) rethink
  int robot_id = 0;
  nh_.getParam("robot_id", robot_id);
  robot_prefix_ = kimera_pgmo::robot_id_to_prefix.at(robot_id);

  // TODO(nathan) think about fixing lcd log path
  config_ = load_config<DsgLcdModuleConfig>(nh_, "");
  lcd_detector_.reset(new lcd::DsgLcdDetector(config_.detector));
}

void DsgLcd::stop() {
  VLOG(2) << "[DSG LCD] stopping lcd!";

  should_shutdown_ = true;
  if (lcd_thread_) {
    VLOG(2) << "[DSG LCD] joining thread";
    lcd_thread_->join();
    lcd_thread_.reset();
    VLOG(2) << "[DSG LCD] joined thread";
  }

  lcd_visualizer_.reset();
}

DsgLcd::~DsgLcd() { stop(); }

void DsgLcd::handleDbowMsg(const pose_graph_tools::BowQuery::ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  bow_messages_.push_back(msg);
}

void DsgLcd::start() {
  bow_sub_ = nh_.subscribe("bow_vectors", 100, &DsgLcd::handleDbowMsg, this);

  if (config_.visualize_dsg_lcd) {
    ros::NodeHandle nh(config_.lcd_visualizer_ns);
    visualizer_queue_.reset(new ros::CallbackQueue());
    nh.setCallbackQueue(visualizer_queue_.get());

    lcd_visualizer_.reset(new lcd::LcdVisualizer(nh, config_.detector.object_radius_m));
    lcd_visualizer_->setGraph(lcd_graph_);
    lcd_visualizer_->setLcdDetector(lcd_detector_.get());
  }

  lcd_thread_.reset(new std::thread(&DsgLcd::runLcd, this));
  LOG(INFO) << "[DSG LCD] LCD started!";
}

std::optional<NodeId> DsgLcd::getLatestAgentId() {
  if (lcd_queue_.empty()) {
    return std::nullopt;
  }

  const auto& node = lcd_graph_->getDynamicNode(lcd_queue_.top())->get();
  const auto prev_time = node.timestamp;
  const bool has_parent = node.hasParent();

  if (!has_parent) {
    LOG(ERROR) << "Found agent node without parent: "
               << NodeSymbol(lcd_queue_.top()).getLabel() << ". Discarding!";
    lcd_queue_.pop();
    return std::nullopt;
  }

  const std::chrono::nanoseconds curr_time(dsg_->last_update_time);
  std::chrono::duration<double> diff_s = curr_time - prev_time;
  // we consider should_shutdown_ here to make sure we're not waiting on popping from
  // the LCD queue while not getting new place messages
  if (!should_shutdown_ && diff_s.count() < config_.lcd_agent_horizon_s) {
    return std::nullopt;
  }

  const bool forced_pop = (diff_s.count() < config_.lcd_agent_horizon_s);
  if (should_shutdown_ && forced_pop) {
    LOG(ERROR) << "Forcing pop of node " << NodeSymbol(lcd_queue_.top()).getLabel()
               << " from lcd queue due to shutdown: parent? "
               << (has_parent ? "yes" : "no") << ", diff: " << diff_s.count() << " / "
               << config_.lcd_agent_horizon_s;
  }

  auto valid_node = lcd_queue_.top();
  lcd_queue_.pop();
  return valid_node;
}

void DsgLcd::runLcd() {
  ros::WallRate r(10);
  while (ros::ok()) {
    assignBowVectors();

    {  // start critical section
      std::unique_lock<std::mutex> lock(dsg_->mutex);
      lcd_graph_->mergeGraph(*dsg_->graph);

      potential_lcd_root_nodes_.insert(potential_lcd_root_nodes_.end(),
                                       dsg_->archived_places.begin(),
                                       dsg_->archived_places.end());
      dsg_->archived_places.clear();
    }  // end critical section

    if (lcd_graph_->getLayer(DsgLayers::PLACES).numNodes() == 0) {
      if (should_shutdown_) {
        break;
      }

      r.sleep();
      continue;
    }

    if (should_shutdown_ && lcd_queue_.empty()) {
      break;
    }

    auto latest_agent = getLatestAgentId();
    if (!latest_agent) {
      r.sleep();
      continue;
    }

    const Eigen::Vector3d latest_pos = lcd_graph_->getPosition(*latest_agent);

    NodeIdSet to_cache;
    auto iter = potential_lcd_root_nodes_.begin();
    while (iter != potential_lcd_root_nodes_.end()) {
      const Eigen::Vector3d pos = lcd_graph_->getPosition(*iter);
      if ((latest_pos - pos).norm() < config_.descriptor_creation_horizon_m) {
        ++iter;
      } else {
        to_cache.insert(*iter);
        iter = potential_lcd_root_nodes_.erase(iter);
      }
    }

    if (!to_cache.empty()) {
      auto curr_time = ros::Time::now();
      lcd_detector_->updateDescriptorCache(*lcd_graph_, to_cache, curr_time.toNSec());
    }

    auto time = lcd_graph_->getDynamicNode(*latest_agent).value().get().timestamp;
    auto results = lcd_detector_->detect(*lcd_graph_, *latest_agent, time.count());
    if (lcd_visualizer_) {
      lcd_visualizer_->setGraphUpdated();
      lcd_visualizer_->redraw();
    }

    if (results.size() == 0) {
      if (!should_shutdown_) {
        r.sleep();
      }
      continue;
    }

    {  // start lcd critical section
      // TODO(nathan) double check logic here
      std::unique_lock<std::mutex> lcd_lock(dsg_->lcd_mutex);
      for (const auto& result : results) {
        dsg_->loop_closures.push(result);
        LOG(WARNING) << "Found valid loop-closure: "
                     << NodeSymbol(result.from_node).getLabel() << " -> "
                     << NodeSymbol(result.to_node).getLabel();
      }
    }  // end lcd critical section

    if (!should_shutdown_) {
      r.sleep();
    }
  }
}

void DsgLcd::assignBowVectors() {
  std::unique_lock<std::mutex> lock(dsg_->mutex);
  const auto& agents = dsg_->graph->getLayer(DsgLayers::AGENTS, robot_prefix_);

  const size_t prior_size = bow_messages_.size();
  auto iter = bow_messages_.begin();
  while (iter != bow_messages_.end()) {
    // TODO(nathan) implicit assumption that gtsam symbol and dsg node symbol are same
    const auto& msg = *iter;
    char prefix = kimera_pgmo::robot_id_to_prefix.at(msg->robot_id);
    NodeSymbol pgmo_key(prefix, msg->pose_id);
    if (dsg_->agent_key_map.count(pgmo_key)) {
      const auto& node = agents.getNodeByIndex(dsg_->agent_key_map.at(pgmo_key))->get();
      lcd_queue_.push(node.id);

      AgentNodeAttributes& attrs = node.attributes<AgentNodeAttributes>();
      attrs.dbow_ids = Eigen::Map<const AgentNodeAttributes::BowIdVector>(
          msg->bow_vector.word_ids.data(), msg->bow_vector.word_ids.size());
      attrs.dbow_values = Eigen::Map<const Eigen::VectorXf>(
          msg->bow_vector.word_values.data(), msg->bow_vector.word_values.size());

      iter = bow_messages_.erase(iter);
    } else {
      ++iter;
    }
  }

  VLOG(3) << "[DSG LCD] " << bow_messages_.size() << " of " << prior_size
          << " bow vectors unassigned";
}

}  // namespace incremental
}  // namespace hydra
