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
#include "hydra/places/traversability_projective_integrator.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <algorithm>
#include <cmath>

#include "hydra/active_window/active_window_output.h"
#include "hydra/input/input_data.h"
#include "hydra/input/sensor.h"

namespace hydra::places {

namespace {

static const auto registration =
    config::RegistrationWithConfig<TraversabilityProcessor,
                                   TraversabilityProjectiveIntegrator,
                                   TraversabilityProjectiveIntegrator::Config>(
        "TraversabilityProjectiveIntegrator");

inline bool contains(const std::vector<int>& labels, int label) {
  return std::find(labels.begin(), labels.end(), label) != labels.end();
}

}  // namespace

void declare_config(TraversabilityProjectiveIntegrator::Config& config) {
  using namespace config;
  name("TraversabilityProjectiveIntegrator::Config");
  enum_field(config.input_image,
             "input_image",
             {{TraversabilityProjectiveIntegrator::InputImage::TRAVERSABILITY,
               "traversability"},
              {TraversabilityProjectiveIntegrator::InputImage::LABEL, "label"}});
  field(config.traversable_labels, "traversable_labels");
  field(config.intraversable_labels, "intraversable_labels");
  field(config.confidence_saturation_count, "confidence_saturation_count");
  field(config.max_range_error,
        "max_range_error",
        config.max_range_error >= 0 ? "m" : "vs");
  field(config.interpolation_method, "interpolation_method");
  field(config.write_debug_value, "write_debug_value");
  field(config.debug_value_scale, "debug_value_scale");

  check(config.confidence_saturation_count, GT, 0, "confidence_saturation_count");
  check(config.max_range_error, NE, 0.0f, "max_range_error");
  check(config.debug_value_scale, GT, 0.0f, "debug_value_scale");
  checkCondition(!config.traversable_labels.empty(), "traversable_labels is empty");
  checkCondition(std::none_of(config.traversable_labels.begin(),
                              config.traversable_labels.end(),
                              [&config](int label) {
                                return contains(config.intraversable_labels, label);
                              }),
                 "traversable_labels and intraversable_labels must be disjoint");
}

TraversabilityProjectiveIntegrator::TraversabilityProjectiveIntegrator(
    const Config& config)
    : config(config::checkValid(config)),
      interpolator_(config.interpolation_method.create()) {}

void TraversabilityProjectiveIntegrator::apply(TraversabilityLayer& layer,
                                               const ActiveWindowOutput& msg) {
  // A collated message can carry several frames; each is an independent observation.
  for (const auto& data : msg.sensor_data) {
    if (!data || labelImage(*data).empty() || data->range_image.empty()) {
      continue;
    }

    // Labels are read as int32. InputData::finalize() normalizes integer
    // traversability images, and label images when the map has semantics.
    if (labelImage(*data).type() != InputData::LabelMatType) {
      LOG_FIRST_N(ERROR, 1) << "[TraversabilityProjectiveIntegrator] Label image must "
                               "be CV_32SC1 (is the input normalized?)";
      continue;
    }

    integrateFrame(layer, *data);
  }

  if (!config.write_debug_value) {
    return;
  }

  // The estimator clears debug values of recomputed blocks, so rewrite all of them.
  for (auto& block : layer) {
    for (auto& voxel : block.voxels) {
      if (voxel.semantic.total() > 0) {
        voxel.debug_value = voxel.semantic.traversability * config.debug_value_scale;
      }
    }
  }
}

const cv::Mat& TraversabilityProjectiveIntegrator::labelImage(
    const InputData& data) const {
  return config.input_image == InputImage::LABEL ? data.label_image
                                                 : data.traversability_image;
}

void TraversabilityProjectiveIntegrator::integrateFrame(TraversabilityLayer& layer,
                                                        const InputData& data) const {
  // getSensorPose() is world_T_body * body_T_sensor, i.e. world_T_sensor.
  const Eigen::Isometry3f sensor_T_world = data.getSensorPose().cast<float>().inverse();
  const auto& sensor = data.getSensor();
  const float max_range_error = config.max_range_error < 0.0f
                                    ? -config.max_range_error * layer.voxel_size
                                    : config.max_range_error;
  const auto saturation = static_cast<float>(config.confidence_saturation_count);
  const auto& labels = labelImage(data);

  // NOTE(aryannav): deliberately does not set `block.updated`. The clusterings iterate
  // `layer.updatedBlocks()`, so flagging blocks the estimator did not recompute would
  // make them re-cluster for no reason. Nothing downstream reads the semantic fields
  // yet, and the visualizer iterates all blocks regardless.
  for (auto& block : layer) {
    const auto origin = block.origin();
    for (size_t x = 0; x < block.voxels_per_side; ++x) {
      const float p_x = origin.x() + (x + 0.5f) * layer.voxel_size;
      for (size_t y = 0; y < block.voxels_per_side; ++y) {
        auto& voxel = block.voxel(x, y);
        // Only cells with an observed surface have a height to project.
        if (!voxel.height) {
          continue;
        }

        const float p_y = origin.y() + (y + 0.5f) * layer.voxel_size;
        const Eigen::Vector3f p_sensor =
            sensor_T_world * Eigen::Vector3f(p_x, p_y, *voxel.height);

        const float voxel_range = p_sensor.norm();
        if (!data.inRange(voxel_range)) {
          continue;
        }

        float u, v;
        if (!sensor.projectPointToImagePlane(p_sensor, u, v)) {
          continue;
        }

        // Weights are computed against the range image, which also validates the
        // pixel(s) are in bounds and finite.
        const auto weights = interpolator_->computeWeights(u, v, data.range_image);
        if (!weights.valid) {
          continue;
        }

        // Visibility gate: the label at this pixel only describes this cell if the
        // measured range agrees with the cell's own range.
        const float measured_range =
            interpolator_->interpolateRange(data.range_image, weights);
        const float range_error = voxel_range - measured_range;
        if (!std::isfinite(range_error) || std::abs(range_error) > max_range_error) {
          continue;
        }

        const auto label = interpolator_->interpolateID(labels, weights);
        auto& semantic = voxel.semantic;
        if (contains(config.traversable_labels, label)) {
          ++semantic.traversable_count;
        } else if (contains(config.intraversable_labels, label)) {
          ++semantic.intraversable_count;
        } else {
          continue;  // unknown label: no evidence either way
        }

        const auto total = static_cast<float>(semantic.total());
        semantic.traversability = semantic.traversable_count / total;
        semantic.confidence = std::min(1.0f, total / saturation);
      }
    }
  }
}

}  // namespace hydra::places
