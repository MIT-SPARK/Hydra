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
#include <pose_graph_tools/pose_graph.h>

#include <memory>

#include "hydra/common/message_queue.h"
#include "hydra/loop_closure/registration_solution.h"

namespace pose_graph_tools {
struct BowQuery;
}  // namespace pose_graph_tools

namespace hydra {

struct BackendInput;
struct LcdInput;
struct FeatureView;

class PipelineQueues {
 public:
  using BowQueue = MessageQueue<std::shared_ptr<const pose_graph_tools::BowQuery>>;
  using LcdQueue = MessageQueue<std::shared_ptr<LcdInput>>;

  ~PipelineQueues();
  static PipelineQueues& instance();
  void clear();

  //! Connection between frontend and backend
  MessageQueue<std::shared_ptr<BackendInput>> backend_queue;
  //! Connection between backend and LCD module
  MessageQueue<lcd::RegistrationSolution> backend_lcd_queue;
  //! Queue for receiving image or pointcloud level descriptors
  MessageQueue<std::unique_ptr<FeatureView>> input_features_queue;
  //! Queue for receiving (timestamped) external loop closures
  MessageQueue<pose_graph_tools::PoseGraph> external_loop_closure_queue;

  //! Optional input queue to LCD module
  LcdQueue::Ptr lcd_queue;
  //! Optional BoW descriptor queue to LCD module
  BowQueue::Ptr bow_queue;

 private:
  PipelineQueues();

  // TODO(nathan) have some sort of config or pull from global config
  // TODO(nathan) fix thread safety (by probably just having a single static instance)
  inline static std::unique_ptr<PipelineQueues> s_instance_;
};

}  // namespace hydra
