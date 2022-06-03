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
#pragma once
#include "hydra_dsg_builder/dsg_lcd_detector.h"
#include "hydra_dsg_builder/lcd_module_config.h"
#include "hydra_dsg_builder/lcd_visualizer.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <pose_graph_tools/BowQuery.h>

#include <memory>
#include <mutex>
#include <thread>

namespace hydra {
namespace incremental {

class DsgLcd {
 public:
  DsgLcd(const ros::NodeHandle& nh, const SharedDsgInfo::Ptr& dsg);

  virtual ~DsgLcd();

  void start();

  void stop();

 private:
  void handleDbowMsg(const pose_graph_tools::BowQuery::ConstPtr& msg);

  void runLcd();

  void assignBowVectors();

  std::optional<NodeId> getLatestAgentId();

 private:
  ros::NodeHandle nh_;
  std::atomic<bool> should_shutdown_{false};

  DsgLcdModuleConfig config_;
  SharedDsgInfo::Ptr dsg_;

  std::priority_queue<NodeId, std::vector<NodeId>, std::greater<NodeId>> lcd_queue_;
  std::unique_ptr<std::thread> lcd_thread_;
  std::unique_ptr<lcd::DsgLcdDetector> lcd_detector_;
  std::unique_ptr<lcd::LcdVisualizer> lcd_visualizer_;
  std::unique_ptr<ros::CallbackQueue> visualizer_queue_;
  DynamicSceneGraph::Ptr lcd_graph_;
  // TODO(nathan) replace with struct passed in through constructor
  char robot_prefix_;

  ros::Subscriber bow_sub_;
  std::list<pose_graph_tools::BowQuery::ConstPtr> bow_messages_;
  std::list<NodeId> potential_lcd_root_nodes_;
};

}  // namespace incremental
}  // namespace hydra
