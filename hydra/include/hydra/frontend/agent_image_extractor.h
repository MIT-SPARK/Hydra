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

#include <spark_dsg/scene_graph_types.h>

#include <Eigen/Geometry>
#include <deque>
#include <filesystem>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <optional>
#include <string>
#include <vector>

#include "hydra/frontend/graph_builder_functor.h"
#include "hydra/frontend/keyframe_gate.h"
#include "hydra/frontend/keyframe_writer.h"
#include "hydra/utils/logging.h"

namespace hydra {

/**
 * @brief Saves color and depth images for agent (pose graph) nodes to disk.
 *
 * Sensor frames of every input are buffered and paired with the agent node created
 * from the same input (agent nodes are stamped with the input timestamp). A keyframe
 * is written for a node when the agent moved or rotated far enough since the last
 * keyframe and the node's image_folder is set to its file prefix (see KeyframeWriter),
 * e.g., `agents/agent_<timestamp_ns>` for the output path `<run>/agents`. Depth is
 * stored as 16-bit millimeters and the metadata includes world_T_body of the frame.
 */
class AgentImageExtractor : public GraphBuilderFunctor {
 public:
  struct Config : public VerbosityConfig {
    Config();

    //! Directory to save keyframe images to (required). Image folders are relative to
    //! its parent directory
    std::filesystem::path image_output_path;
    //! Name of the sensor to save images from (empty accepts any sensor)
    std::string sensor_name;
    //! Motion required between keyframes
    KeyframeGate::Config gate{1.0, 30.0};
    //! Maximum time difference [s] between an agent node and the paired sensor frame
    double max_pairing_time_diff_s = 0.05;
    //! Number of recent sensor frames retained for pairing. Each entry owns a copy of
    //! the images it will write, so this bounds the memory of the extractor
    size_t max_buffered_frames = 15;
    //! Number of updates an agent node may wait for its sensor frame before it is
    //! skipped so that later nodes can be processed
    size_t max_deferred_updates = 10;
  } const config;

  explicit AgentImageExtractor(const Config& config);

  //! Buffers the sensor frames of all inputs (deep copies in the on-disk format)
  void call(const ActiveWindowOutput& msg,
            SharedDsgInfo& dsg,
            FrontendOutput& output,
            const VolumetricWindow* window) override;

  //! Pairs the new agent nodes with buffered frames and writes keyframes
  void callPostUpdate(SharedDsgInfo& dsg, FrontendOutput& output) override;

 private:
  //! One buffered sensor frame (owning its pixel buffers)
  struct BufferedFrame {
    uint64_t timestamp_ns = 0;
    cv::Mat color;
    cv::Mat depth;
    Eigen::Isometry3d world_T_body = Eigen::Isometry3d::Identity();
  };

  //! Keyframe selected for an agent node
  struct Keyframe {
    spark_dsg::NodeId node;
    uint64_t node_ns;
    BufferedFrame frame;
  };

  //! Index of the buffered frame nearest to timestamp_ns at or after search_start
  std::optional<size_t> findFrame(const std::vector<BufferedFrame>& frames,
                                  size_t search_start,
                                  uint64_t timestamp_ns) const;

  //! Pair pending agent nodes with frames and select keyframes
  std::vector<Keyframe> selectKeyframes(const spark_dsg::SceneGraph& graph,
                                        const std::vector<BufferedFrame>& frames);

  KeyframeGate gate_;
  KeyframeWriter writer_;

  //! New agent nodes that were not paired with a frame yet (oldest first)
  std::deque<spark_dsg::NodeId> pending_nodes_;
  //! Number of updates the oldest pending node has been waiting for its frame
  size_t deferred_count_ = 0;
  //! Timestamp of the newest agent node already decided on
  uint64_t last_decided_ns_ = 0;

  //! Recent sensor frames (oldest first) and the calibration of their camera
  std::mutex frame_mutex_;
  std::deque<BufferedFrame> frames_;
  std::optional<CameraCalib> calib_;
};

void declare_config(AgentImageExtractor::Config& config);

}  // namespace hydra
