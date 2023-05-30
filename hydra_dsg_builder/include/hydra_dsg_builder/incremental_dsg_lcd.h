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
#include "hydra_dsg_builder/incremental_types.h"
#include "hydra_dsg_builder/lcd_module_config.h"
#include "hydra_dsg_builder/shared_module_state.h"

#include <hydra_utils/robot_prefix_config.h>

#include <memory>
#include <thread>

namespace hydra {
namespace incremental {

class DsgLcd {
 public:
  DsgLcd(const RobotPrefixConfig& prefix,
         const DsgLcdModuleConfig& config,
         const SharedDsgInfo::Ptr& dsg,
         const SharedModuleState::Ptr& state);

  virtual ~DsgLcd();

  void start();

  void stop();

  void save(const std::string& output_path);

  void spin();

  void spinOnce(bool force_update);

 protected:
  void spinOnceImpl(bool force_update);

  size_t processFrontendOutput();

  NodeIdSet getPlacesToCache(const Eigen::Vector3d& agent_pos);

  std::optional<NodeId> getQueryAgentId(size_t timestamp_ns);

 protected:
  std::atomic<bool> should_shutdown_{false};
  std::unique_ptr<std::thread> spin_thread_;

  RobotPrefixConfig prefix_;
  DsgLcdModuleConfig config_;
  SharedDsgInfo::Ptr dsg_;
  SharedModuleState::Ptr state_;

  std::priority_queue<NodeId, std::vector<NodeId>, std::greater<NodeId>> agent_queue_;
  std::list<NodeId> potential_lcd_root_nodes_;

  std::unique_ptr<lcd::DsgLcdDetector> lcd_detector_;
  DynamicSceneGraph::Ptr lcd_graph_;
};

}  // namespace incremental
}  // namespace hydra
