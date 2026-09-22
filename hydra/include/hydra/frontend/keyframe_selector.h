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

#include <memory>

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
  } const config;

  explicit DistancePolicy(const Config& config);
  virtual ~DistancePolicy() = default;

 protected:
  bool shouldAddImpl(const InputData& candidate,
                     const std::list<InputData::ConstPtr>& keyframes,
                     std::string& reason) const override;
};

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
  } const config;

  KeyframeSelector(const Config& config);

  void call(const ActiveWindowOutput& msg,
            SharedDsgInfo& dsg,
            FrontendOutput& output,
            const VolumetricWindow* window) override;

  void callPostUpdate(SharedDsgInfo& dsg, FrontendOutput& output) override;

 protected:
  void archiveKeyframes(const ActiveWindowOutput& output,
                        const VolumetricWindow& window);

  void cleanInactive(const spark_dsg::SceneGraph& graph);

  Sink::List sinks_;
  std::unique_ptr<KeyframePolicy> policy_;
  std::unique_ptr<FeatureSelector> feature_selector_;

  std::list<InputData::ConstPtr> keyframes_;
  std::list<InputData::ConstPtr> to_archive_;
  mutable std::map<std::string, ActiveWindowTracker> active_window_;
};

void declare_config(KeyframeSelector::Config& config);

}  // namespace hydra
