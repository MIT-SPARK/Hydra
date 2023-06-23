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
#include <hydra/common/shared_module_state.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <pose_graph_tools/BowQueries.h>

#include "hydra_ros/pipeline/ros_backend.h"
#include "hydra_ros/pipeline/ros_frontend.h"
#include "hydra_ros/pipeline/ros_reconstruction.h"

namespace hydra {

struct HydraRosConfig {
  bool enable_lcd = false;
  bool use_ros_backend = false;
  bool do_reconstruction = true;
  bool enable_frontend_output = true;
  double frontend_mesh_separation_s = 0.0;
};

struct HydraRosPipeline {
  explicit HydraRosPipeline(const ros::NodeHandle& nh, int robot_id = 0);

  void start();

  void stop();

  void save(const std::string& output_path);

  void bowCallback(const pose_graph_tools::BowQueries::ConstPtr& msg);

  void sendFrontendOutput(const DynamicSceneGraph& graph,
                          const BackendInput& backend_input,
                          uint64_t timestamp_ns);

  void sendFrontendGraph(const DynamicSceneGraph& graph, uint64_t timestamp_ns);

  ros::NodeHandle nh;

  HydraRosConfig config;
  RobotPrefixConfig prefix;
  SharedDsgInfo::Ptr frontend_dsg;
  SharedDsgInfo::Ptr backend_dsg;
  SharedModuleState::Ptr shared_state;

  std::shared_ptr<ReconstructionModule> reconstruction;
  std::shared_ptr<FrontendModule> frontend;
  std::shared_ptr<BackendModule> backend;
  std::shared_ptr<RosBackendVisualizer> backend_visualizer;
  std::shared_ptr<LoopClosureModule> lcd;

  std::unique_ptr<DsgSender> dsg_sender;
  ros::Publisher mesh_graph_pub;
  ros::Publisher mesh_update_pub;
  ros::Subscriber bow_sub;
};

}  // namespace hydra
