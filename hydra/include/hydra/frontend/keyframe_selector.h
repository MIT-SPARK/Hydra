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
#include <config_utilities/virtual_config.h>

#include <condition_variable>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "hydra/common/output_sink.h"
#include "hydra/frontend/feature_selector.h"
#include "hydra/frontend/graph_builder_functor.h"
#include "hydra/utils/active_window_tracker.h"
#include "hydra/utils/logging.h"

namespace hydra {

class KeyframePolicy {
 public:
  virtual ~KeyframePolicy() = default;
  bool shouldAdd(const InputData::ConstPtr& candidate,
                 const std::list<InputData::ConstPtr>& keyframes,
                 std::string& reason) const;

 protected:
  virtual bool shouldAddImpl(const InputData& candidate,
                             const std::list<InputData::ConstPtr>& keyframes,
                             std::string& reason) const = 0;
};

class DistancePolicy : public KeyframePolicy {
 public:
  struct Config {
    //! @brief Minimum between pose norm to add new keyframe
    double min_pose_separation = 1.0;
    //! @brief Weighting between rotation (frobenius) norm and translation (l2) norm
    double rotation_separation_weight = 0.1;
    //! @brief Minimum time separation to add new keyframe
    double min_time_separation_s = 0.5;
    //! @brief Add keyframe after this much time even without motion (0 = off)
    double max_time_separation_s = 0.0;
  } const config;

  explicit DistancePolicy(const Config& config);
  virtual ~DistancePolicy() = default;

 protected:
  bool shouldAddImpl(const InputData& candidate,
                     const std::list<InputData::ConstPtr>& keyframes,
                     std::string& reason) const override;
};

void declare_config(DistancePolicy::Config& config);

class KeyframeSelector : public GraphBuilderFunctor {
 public:
  using Keyframes = std::list<InputData::ConstPtr>;
  using Sink = OutputSink<uint64_t, const Keyframes&>;

  struct Config : public VerbosityConfig {
    Config();

    //! Method for extracting keyframes from incoming data
    config::VirtualConfig<KeyframePolicy> keyframe_policy;
    //! Method to control mapping from views to resulting feature for a node
    config::VirtualConfig<FeatureSelector> feature_selector;
    //! Layers to assign views for
    std::vector<std::string> layers{spark_dsg::DsgLayers::PLACES,
                                    spark_dsg::DsgLayers::MESH_PLACES};
    //! Output sinks and visualization
    std::vector<Sink::Factory> sinks;
    //! Maximum number of active keyframes held in memory (0 = unbounded). Keyframes
    //! only leave the active set when they leave the spatial window, so a stationary
    //! robot with DistancePolicy::max_time_separation_s would otherwise grow it without
    //! bound. The oldest are evicted first: to the archive used for feature assignment
    //! when a feature selector is configured, else dropped (they are already queued for
    //! saving).
    size_t max_active_keyframes = 50;
    //! Directory to save accepted keyframes to (empty = off): <timestamp_ns>.zip for
    //! the first sensor saved, <timestamp_ns>_<sensor name>.zip for any other sensor
    //! (name characters outside [A-Za-z0-9_.-] replaced by '_')
    std::filesystem::path log_path;
    //! Images to keep in saved keyframes (color, color_mask, depth, range, labels,
    //! instances, vertices, traversability, features)
    std::vector<std::string> save_fields{"color", "depth"};
    //! Maximum number of keyframes waiting to be saved (0 = unbounded)
    size_t max_queue_size = 100;
    //! Encoding of saved keyframes (EXR vertex maps are compressed by default)
    InputData::SaveOptions save_options{InputData::SaveOptions::FloatCompression::ZIP};
  } const config;

  KeyframeSelector(const Config& config);

  virtual ~KeyframeSelector();

  void call(const ActiveWindowOutput& msg,
            SharedDsgInfo& dsg,
            FrontendOutput& output,
            const VolumetricWindow* window) override;

  void callPostUpdate(SharedDsgInfo& dsg, FrontendOutput& output) override;

  void save(const DataDirectory& output) override;

  //! Block until all queued keyframes are written to disk
  void flush();

  //! Shallow copy of the input with only the requested images (and features) present;
  //! kept images share their pixel buffers with the input
  static InputData::Ptr selectFields(const InputData& data,
                                     const std::vector<std::string>& fields);

 protected:
  void queueForSaving(const InputData::ConstPtr& data);

  void saveSpin();

  //! File name of a saved keyframe (save thread only)
  std::string archiveName(const InputData& data);

  void archiveKeyframes(const ActiveWindowOutput& output,
                        const VolumetricWindow& window);

  //! Move a keyframe out of the active set (to the archive with a feature selector)
  void releaseKeyframe(const InputData::ConstPtr& frame);

  //! Evict the oldest keyframes beyond max_active_keyframes
  void limitActiveKeyframes();

  void cleanInactive(const spark_dsg::SceneGraph& graph);

  Sink::List sinks_;
  std::unique_ptr<KeyframePolicy> policy_;
  std::unique_ptr<FeatureSelector> feature_selector_;

  std::list<InputData::ConstPtr> keyframes_;
  std::list<InputData::ConstPtr> to_archive_;
  mutable std::map<std::string, ActiveWindowTracker> active_window_;

  // background keyframe saving
  std::mutex save_mutex_;
  std::condition_variable save_cv_;
  std::deque<InputData::ConstPtr> save_queue_;
  bool saving_ = false;
  bool should_shutdown_ = false;
  std::unique_ptr<std::thread> save_thread_;
  //! Sensor whose keyframes are saved without a sensor suffix (the first one saved)
  std::optional<std::string> primary_sensor_;
};

void declare_config(KeyframeSelector::Config& config);

}  // namespace hydra
