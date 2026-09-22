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
#include "hydra/frontend/keyframe_selector.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <spark_dsg/node_attributes.h>
#include <spark_dsg/printing.h>

#include "hydra/active_window/volumetric_window.h"
#include "hydra/utils/printing.h"
#include "hydra/utils/timing_utilities.h"

using namespace spark_dsg;

namespace hydra {
namespace {

static const auto policy_registration =
    config::RegistrationWithConfig<KeyframePolicy,
                                   DistancePolicy,
                                   DistancePolicy::Config>("DistancePolicy");

static const auto functor_registration =
    config::RegistrationWithConfig<GraphBuilderFunctor,
                                   KeyframeSelector,
                                   KeyframeSelector::Config>("KeyframeSelector");

std::string showVec(const Eigen::MatrixXf& vec, size_t max_length = 100) {
  if (vec.rows() * vec.cols() == 0) {
    return "[]";
  }

  std::stringstream ss;
  ss << "[";
  for (int i = 0; i < vec.rows(); ++i) {
    ss << std::setprecision(3) << vec(i, 0);
    if (i < vec.rows() - 1) {
      ss << ", ";
    }

    if (ss.str().size() >= max_length) {
      ss << "...";
      break;
    }
  }
  ss << "]";

  return ss.str();
}

bool isVisible(const FeatureSelector& selector,
               const FeatureView& view,
               const LayerView& nodes) {
  for (const auto& node : nodes) {
    auto attrs = node.tryAttributes<SemanticNodeAttributes>();
    if (!attrs) {
      continue;
    }

    if (selector.nodeInView(view, *attrs)) {
      return true;
    }
  }

  return false;
}

}  // namespace

using hydra::timing::ScopedTimer;

void declare_config(DistancePolicy::Config& config) {
  using namespace config;
  name<DistancePolicy::Config>();
  field(config.min_pose_separation, "min_pose_separation");
  field(config.rotation_separation_weight, "rotation_separation_weight");
  field(config.min_time_separation_s, "min_time_separation_s");
  check(config.min_pose_separation, GE, 0.0, "rotation_separation_weight");
  check(config.rotation_separation_weight, GE, 0.0, "rotation_separation_weight");
  check(config.min_time_separation_s, GE, 0.0, "rotation_separation_weight");
}

bool KeyframePolicy::shouldAdd(const InputData::ConstPtr& candidate,
                               const std::list<InputData::ConstPtr>& keyframes,
                               std::string& reason) const {
  if (!candidate) {
    return false;
  }

  if (keyframes.empty()) {
    return true;
  }

  return shouldAddImpl(*candidate, keyframes, reason);
}

DistancePolicy::DistancePolicy(const Config& config)
    : config(config::checkValid(config)) {}

bool DistancePolicy::shouldAddImpl(const InputData& candidate,
                                   const std::list<InputData::ConstPtr>& keyframes,
                                   std::string& reason) const {
  const auto& to_check = *keyframes.back();
  using std::chrono::duration_cast;
  const auto curr_stamp = std::chrono::nanoseconds(candidate.timestamp_ns);
  const auto last_stamp = std::chrono::nanoseconds(to_check.timestamp_ns);

  const auto diff_ns = curr_stamp - last_stamp;
  const auto diff_s = duration_cast<std::chrono::duration<double>>(diff_ns);
  if (config.min_time_separation_s && diff_s.count() < config.min_time_separation_s) {
    reason = std::format("Dropped candidate @ {} [ns] with time diff {} < {} [s]",
                         candidate.timestamp_ns,
                         diff_s.count(),
                         config.min_time_separation_s);
    return false;
  }

  const Eigen::Isometry3d pose_diff =
      candidate.world_T_body.inverse() * to_check.world_T_body;
  const auto diff_t = pose_diff.translation().norm();
  const auto diff_r = pose_diff.rotation().norm();
  const auto diff_p = diff_t + config.rotation_separation_weight * diff_r;
  if (config.min_pose_separation && diff_p < config.min_pose_separation) {
    reason = std::format("Dropped candidate @ {} [ns] with pose diff {} < {}",
                         candidate.timestamp_ns,
                         diff_p,
                         config.min_pose_separation);
    return false;
  }

  return true;
}

void declare_config(KeyframeSelector::Config& config) {
  using namespace config;
  name("KeyframeSelector::Config");
  base<VerbosityConfig>(config);
  field(config.keyframe_policy, "keyframe_policy");
  config.feature_selector.setOptional();
  field(config.feature_selector, "feature_selector");
  field(config.layers, "layers");
  field(config.sinks, "sinks");
}

KeyframeSelector::Config::Config()
    : VerbosityConfig(VerbosityConfig::default_verbosity("keyframes")),
      keyframe_policy(DistancePolicy::Config()) {}

KeyframeSelector::KeyframeSelector(const Config& config)
    : config(config::checkValid(config)),
      sinks_(Sink::instantiate(config.sinks)),
      policy_(config.keyframe_policy.create()),
      feature_selector_(config.feature_selector.create()) {
  for (const auto& layer : config.layers) {
    active_window_.emplace(layer, ActiveWindowTracker());
  }
}

void KeyframeSelector::call(const ActiveWindowOutput& input,
                            SharedDsgInfo&,
                            FrontendOutput&,
                            const VolumetricWindow* window) {
  ScopedTimer timer("frontend/update_keyframes", input.timestamp_ns);

  size_t num_added = 0;
  for (const auto& data : input.sensor_data) {
    std::string reason;
    if (policy_->shouldAdd(data, keyframes_, reason)) {
      keyframes_.push_back(data);
      ++num_added;
    } else if (!reason.empty()) {
      MLOG(2) << reason;
    }
  }

  // TODO(nathan) add saving keyframes to disk (could be sink)

  MLOG(2) << "Got " << num_added << " new views!";
  if (window) {
    archiveKeyframes(input, *window);
  }

  Sink::callAll(sinks_, input.timestamp_ns, keyframes_);
}

void KeyframeSelector::archiveKeyframes(const ActiveWindowOutput& msg,
                                        const VolumetricWindow& window) {
  auto iter = keyframes_.begin();
  while (iter != keyframes_.end()) {
    const auto& frame = *iter;
    const Eigen::Vector3d pos = frame->world_T_body.translation();
    const auto stamp = frame->timestamp_ns;

    const auto fmt = getDefaultFormat(3);
    MLOG(4) << "view @ " << stamp << "[ns]: " << pos.format(fmt) << " vs. "
            << msg.world_T_body().translation().format(fmt);

    if (!window.inBounds(msg.timestamp_ns, msg.world_T_body(), stamp, pos)) {
      MLOG(3) << "Archival candidate found @ " << stamp << " [ns]";
      to_archive_.push_back(*iter);
      iter = keyframes_.erase(iter);
      continue;
    }

    ++iter;
  }
}

void KeyframeSelector::cleanInactive(const SceneGraph& graph) {
  // drop keyframes that no longer observe any inactive nodes
  auto iter = to_archive_.begin();
  while (iter != to_archive_.end()) {
    bool visible = false;
    const FeatureView view(**iter);
    for (auto& [name, tracker] : active_window_) {
      auto layer = graph.findLayer(name);
      if (!layer) {
        continue;
      }

      visible = isVisible(*feature_selector_, view, tracker.view(*layer));
      if (visible) {
        break;
      }
    }

    if (!visible) {
      iter = to_archive_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void KeyframeSelector::callPostUpdate(SharedDsgInfo& dsg, FrontendOutput&) {
  if (!feature_selector_) {
    return;
  }

  for (auto& [name, tracker] : active_window_) {
    tracker.clear();
  }

  cleanInactive(*dsg.graph);

  const auto num_frames = keyframes_.size() + to_archive_.size();
  if (!num_frames) {
    MLOG(2) << "Skipping feature assignment without any active keyframes";
    return;
  }

  MLOG(2) << "Assigning features with " << num_frames << " active keyframe(s)";

  std::vector<FeatureView> views;
  views.reserve(num_frames);
  for (const auto& frame : keyframes_) {
    views.emplace_back(*frame);
  }

  for (const auto& frame : to_archive_) {
    views.emplace_back(*frame);
  }

  for (auto& [name, layer_tracker] : active_window_) {
    auto layer = dsg.graph->findLayer(name);
    if (!layer) {
      LOG(WARNING) << config.prefix << "Skipping unknown layer: '" << name << "'";
      continue;
    }

    size_t num_seen = 0;
    size_t num_assigned = 0;
    for (const auto& node : layer_tracker.view(*layer)) {
      auto attrs = node.tryAttributes<SemanticNodeAttributes>();
      if (!attrs) {
        LOG(ERROR) << config.prefix << "Invalid node " << NodeSymbol(node.id).str();
        continue;
      }

      ++num_seen;
      if (feature_selector_->select(views, *attrs)) {
        MLOG(5) << "node " << NodeSymbol(node.id).str() << ": "
                << showVec(attrs->semantic_feature);
        ++num_assigned;
      }
    }

    if (num_seen) {
      MLOG(2) << "Assigned features to " << num_assigned << " / " << num_seen
              << " nodes for layer '" << name << "'";
    }
  }
}

}  // namespace hydra
