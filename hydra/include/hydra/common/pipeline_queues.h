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

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "hydra/common/message_queue.h"
#include "hydra/common/sub_keyframes.h"
#include "hydra/loop_closure/registration_solution.h"

namespace hydra {

struct FrontendOutput;
struct ImageInputPacket;
class Sensor;

//! Color and depth images received at the full input rate
struct SubKeyframeInput {
  //! Sensor the images belong to
  std::shared_ptr<const Sensor> sensor;
  //! Timestamp of the images
  uint64_t timestamp_ns = 0;
  //! Parses the received images (deferred so that only selected images are parsed)
  std::function<std::shared_ptr<const ImageInputPacket>()> parse;
};

class PipelineQueues {
 public:
  ~PipelineQueues();

  static PipelineQueues& instance();

  void clear();

  //! Connection between frontend and backend
  MessageQueue<std::shared_ptr<const FrontendOutput>> backend_queue;
  //! Connection between backend and LCD module
  MessageQueue<lcd::RegistrationSolution> backend_lcd_queue;
  //! Queue for receiving (timestamped) external loop closures
  MessageQueue<pose_graph_tools::PoseGraph> external_loop_closure_queue;
  //! Full-rate images for sub-keyframes (see acceptsSubKeyframes)
  MessageQueue<SubKeyframeInput> subkeyframe_queue;
  //! Sub-keyframe node requests drained by the frontend (which owns all mutation of
  //! the frontend graph)
  MessageQueue<SubKeyframeRequest> subkeyframe_node_queue;

  /**
   * @brief Start accepting sub-keyframe images (see acceptsSubKeyframes)
   * @param max_queue_size Maximum size of the sub-keyframe queues
   * @param sensor_name Sensor to accept images from (empty accepts any sensor)
   */
  void enableSubKeyframes(size_t max_queue_size, const std::string& sensor_name);

  //! @brief Stop accepting sub-keyframe images and clear the sub-keyframe queues
  void disableSubKeyframes();

  //! @brief Whether images of a sensor should be pushed to subkeyframe_queue
  bool acceptsSubKeyframes(const std::string& sensor_name) const;

 private:
  PipelineQueues();

  std::atomic<bool> subkeyframes_enabled_{false};
  mutable std::mutex subkeyframe_mutex_;
  std::string subkeyframe_sensor_;

  // TODO(nathan) fix thread safety (by probably just having a single static instance)
  inline static std::unique_ptr<PipelineQueues> s_instance_;
};

}  // namespace hydra
